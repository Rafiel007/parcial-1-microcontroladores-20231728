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
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 256U
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1U)
#define UART_LINE_MAX_LENGTH 63U
#define UART_TX_BUFFER_SIZE 512U
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t dutyCycle = 10U;
uint16_t dutyMin = 10U;
uint16_t dutyMax = 90U;
static uint32_t fixedPulse = 0U;
static uint8_t rampIncreasing = 1U;
static uint8_t rampMode = 0U;
static uint8_t paused = 0U;
static volatile uint32_t appTickMs = 0U;
static uint32_t lastDutyUpdateMs = 0U;
static uint32_t lastAutoLedToggleMs = 0U;
static uint32_t lastPauseLedToggleMs = 0U;
static volatile uint8_t buttonEdgePending = 0U;
static GPIO_PinState buttonStableState = GPIO_PIN_SET;
static GPIO_PinState buttonCandidateState = GPIO_PIN_SET;
static uint8_t buttonCandidateActive = 0U;
static uint32_t buttonCandidateSinceMs = 0U;
static uint32_t buttonPressStartMs = 0U;
static uint8_t buttonLongPressHandled = 0U;
static uint32_t acceptedShortPresses = 0U;
static uint32_t acceptedLongPresses = 0U;
static uint8_t uartRxByte = 0U;
static uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uartRxHead = 0U;
static volatile uint16_t uartRxTail = 0U;
static volatile uint8_t uartRxOverflowPending = 0U;
static volatile uint32_t uartRxOverflowCount = 0U;
static volatile uint32_t uartRxErrorCount = 0U;
static char uartLineBuffer[UART_LINE_MAX_LENGTH + 1U];
static char uartLastLine[UART_LINE_MAX_LENGTH + 1U];
static uint8_t uartLineLength = 0U;
static uint8_t uartLineDiscarding = 0U;
static uint8_t uartSkipLfAfterCr = 0U;
static uint32_t uartCompletedLineCount = 0U;
static uint32_t uartLineOverflowCount = 0U;
static uint16_t pwmFrequencyHz = 1000U;
static char uartTxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t uartTxHead = 0U;
static volatile uint16_t uartTxTail = 0U;
static volatile uint8_t uartTxBusy = 0U;
static uint8_t uartTxByteInFlight = 0U;
static volatile uint32_t uartTxOverflowCount = 0U;
static volatile uint32_t uartTxErrorCount = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void Button_Process(uint32_t now);
static uint8_t Button_TakeEdgePending(void);
static uint32_t PWM_DutyToPulse(uint16_t dutyPercent);
static void PWM_ApplyCurrentMode(void);
static void Button_HandleShortPress(uint32_t now);
static void Button_HandleLongPress(uint32_t now);
static uint8_t UART_TakeRxByte(uint8_t *byte);
static void UART_ProcessRx(void);
static uint8_t UART_QueueText(const char *text);
static void UART_TxStartNext(void);
static void UART_ProcessCommand(char *line);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t PWM_DutyToPulse(uint16_t dutyPercent)
{
  return ((htim2.Init.Period + 1U) * dutyPercent) / 100U;
}

static uint8_t UART_TakeRxByte(uint8_t *byte)
{
  uint16_t tail = uartRxTail;
  if (tail == uartRxHead)
  {
    return 0U;
  }

  *byte = uartRxBuffer[tail];
  uartRxTail = (tail + 1U) & UART_RX_BUFFER_MASK;
  return 1U;
}

static void UART_TxStartNext(void)
{
  uint32_t interruptMask = __get_PRIMASK();
  __disable_irq();

  if ((uartTxBusy != 0U) || (uartTxTail == uartTxHead))
  {
    __set_PRIMASK(interruptMask);
    return;
  }

  uartTxByteInFlight = (uint8_t)uartTxBuffer[uartTxTail];
  uartTxTail = (uartTxTail + 1U) & UART_TX_BUFFER_MASK;
  uartTxBusy = 1U;
  __set_PRIMASK(interruptMask);

  if (HAL_UART_Transmit_IT(&huart2, &uartTxByteInFlight, 1U) != HAL_OK)
  {
    interruptMask = __get_PRIMASK();
    __disable_irq();
    uartTxTail = (uartTxTail - 1U) & UART_TX_BUFFER_MASK;
    uartTxBusy = 0U;
    uartTxErrorCount++;
    __set_PRIMASK(interruptMask);
  }
}

static uint8_t UART_QueueText(const char *text)
{
  size_t length = strlen(text);
  uint32_t interruptMask = __get_PRIMASK();
  __disable_irq();

  uint16_t used = (uartTxHead - uartTxTail) & UART_TX_BUFFER_MASK;
  uint16_t freeSpace = UART_TX_BUFFER_MASK - used;
  if (length > freeSpace)
  {
    uartTxOverflowCount++;
    __set_PRIMASK(interruptMask);
    return 0U;
  }

  uint16_t head = uartTxHead;
  for (size_t index = 0U; index < length; index++)
  {
    uartTxBuffer[head] = text[index];
    head = (head + 1U) & UART_TX_BUFFER_MASK;
  }
  uartTxHead = head;
  __set_PRIMASK(interruptMask);

  UART_TxStartNext();
  return 1U;
}

static void UART_ProcessCommand(char *line)
{
  char *command = line;
  while (*command == ' ')
  {
    command++;
  }

  size_t length = strlen(command);
  while ((length > 0U) && (command[length - 1U] == ' '))
  {
    command[--length] = '\0';
  }

  if (length == 0U)
  {
    return;
  }

  char *argument = command;
  while ((*argument != '\0') && (*argument != ' '))
  {
    argument++;
  }
  if (*argument != '\0')
  {
    *argument++ = '\0';
    while (*argument == ' ')
    {
      argument++;
    }
  }

  if ((strcmp(command, "HELP") == 0) ||
      (strcmp(command, "STATUS") == 0))
  {
    if (*argument != '\0')
    {
      UART_QueueText("ERR ARG\r\n");
      return;
    }

    if (strcmp(command, "HELP") == 0)
    {
      UART_QueueText("OK HELP\r\nHELP\r\nSTATUS\r\n");
      return;
    }

    const char *mode = (paused != 0U) ? "PAUSA" :
                       (rampMode != 0U) ? "AUTO" : "MANUAL";
    uint32_t periodCounts = htim2.Init.Period + 1U;
    uint32_t manualDuty = (periodCounts != 0U) ?
                          (fixedPulse * 100U) / periodCounts : 0U;
    uint32_t appliedDuty = (paused != 0U) ? 0U :
                           (rampMode != 0U) ? dutyCycle : manualDuty;
    uint32_t errorCount = uartRxOverflowCount + uartRxErrorCount +
                          uartLineOverflowCount + uartTxOverflowCount +
                          uartTxErrorCount;
    char response[192];
    int responseLength = snprintf(response, sizeof(response),
                                  "OK\n MODE=%s FREQ=%u DUTY=%lu MANUAL=%lu "
                                  "UPTIME_MS=%lu SHORT=%lu LONG=%lu ERR=%lu\r\n",
                                  mode, (unsigned int)pwmFrequencyHz,
                                  (unsigned long)appliedDuty,
                                  (unsigned long)manualDuty,
                                  (unsigned long)appTickMs,
                                  (unsigned long)acceptedShortPresses,
                                  (unsigned long)acceptedLongPresses,
                                  (unsigned long)errorCount);
    if ((responseLength > 0) && ((size_t)responseLength < sizeof(response)))
    {
      UART_QueueText(response);
    }
    else
    {
      uartTxErrorCount++;
    }
    return;
  }

  UART_QueueText("ERR COMMAND\r\n");
}

static void UART_ProcessRx(void)
{
  uint32_t interruptMask = __get_PRIMASK();
  __disable_irq();
  uint8_t overflowPending = uartRxOverflowPending;
  uartRxOverflowPending = 0U;
  __set_PRIMASK(interruptMask);

  if (overflowPending != 0U)
  {
    uartLineLength = 0U;
    uartLineDiscarding = 1U;
    uartRxErrorCount++;
  }

  uint8_t byte;
  while (UART_TakeRxByte(&byte) != 0U)
  {
    if ((uartSkipLfAfterCr != 0U) && (byte == '\n'))
    {
      uartSkipLfAfterCr = 0U;
      continue;
    }
    uartSkipLfAfterCr = 0U;

    if ((byte == '\r') || (byte == '\n'))
    {
      if (byte == '\r')
      {
        uartSkipLfAfterCr = 1U;
      }

      if (uartLineDiscarding != 0U)
      {
        uartLineDiscarding = 0U;
        UART_QueueText("ERR OVERFLOW\r\n");
      }
      else
      {
        uartLineBuffer[uartLineLength] = '\0';
        for (uint8_t index = 0U; index <= uartLineLength; index++)
        {
          uartLastLine[index] = uartLineBuffer[index];
        }
        uartCompletedLineCount++;
        UART_ProcessCommand(uartLineBuffer);
      }
      uartLineLength = 0U;
      continue;
    }

    if (uartLineDiscarding == 0U)
    {
      if (uartLineLength < UART_LINE_MAX_LENGTH)
      {
        uartLineBuffer[uartLineLength++] = (char)byte;
      }
      else
      {
        uartLineLength = 0U;
        uartLineDiscarding = 1U;
        uartLineOverflowCount++;
      }
    }
  }
}

static uint8_t Button_TakeEdgePending(void)
{
  uint32_t interruptMask = __get_PRIMASK();
  __disable_irq();
  uint8_t edgePending = buttonEdgePending;
  buttonEdgePending = 0U;
  __set_PRIMASK(interruptMask);
  return edgePending;
}

static void PWM_ApplyCurrentMode(void)
{
  if (paused != 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  }
  else if (rampMode != 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                          PWM_DutyToPulse(dutyCycle));
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, fixedPulse);
  }
}

static void Button_HandleShortPress(uint32_t now)
{
  if (paused != 0U)
  {
    return;
  }

  acceptedShortPresses++;
  if (rampMode == 0U)
  {
    rampMode = 1U;
    dutyCycle = dutyMin;
    rampIncreasing = 1U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    lastAutoLedToggleMs = now;
  }
  else
  {
    rampMode = 0U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }

  lastDutyUpdateMs = now;
  PWM_ApplyCurrentMode();
}

static void Button_HandleLongPress(uint32_t now)
{
  acceptedLongPresses++;
  paused = (paused == 0U) ? 1U : 0U;
  lastDutyUpdateMs = now;

  if (paused != 0U)
  {
    lastPauseLedToggleMs = now;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    lastAutoLedToggleMs = now;
  }

  PWM_ApplyCurrentMode();
}

static void Button_Process(uint32_t now)
{
  GPIO_PinState pinState = HAL_GPIO_ReadPin(BotonAzul_GPIO_Port, BotonAzul_Pin);

  if (Button_TakeEdgePending() != 0U)
  {
    buttonCandidateState = pinState;
    buttonCandidateSinceMs = now;
    buttonCandidateActive = 1U;
  }

  if (buttonCandidateActive != 0U)
  {
    if (pinState != buttonCandidateState)
    {
      buttonCandidateState = pinState;
      buttonCandidateSinceMs = now;
    }
    else if ((uint32_t)(now - buttonCandidateSinceMs) >= 30U)
    {
      buttonCandidateActive = 0U;
      if (buttonCandidateState != buttonStableState)
      {
        buttonStableState = buttonCandidateState;
        if (buttonStableState == GPIO_PIN_RESET)
        {
          buttonPressStartMs = now;
          buttonLongPressHandled = 0U;
        }
        else if (buttonLongPressHandled == 0U)
        {
          uint32_t pressDuration = (uint32_t)(now - buttonPressStartMs);
          if ((pressDuration >= 30U) && (pressDuration < 1500U))
          {
            Button_HandleShortPress(now);
          }
        }
      }
    }
  }

  if ((buttonStableState == GPIO_PIN_RESET) &&
      (buttonLongPressHandled == 0U) &&
      ((uint32_t)(now - buttonPressStartMs) >= 1500U))
  {
    buttonLongPressHandled = 1U;
    Button_HandleLongPress(now);
  }

  if ((paused != 0U) &&
      ((uint32_t)(now - lastPauseLedToggleMs) >= 125U))
  {
    lastPauseLedToggleMs = now;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
  else if ((paused == 0U) && (rampMode != 0U) &&
           ((uint32_t)(now - lastAutoLedToggleMs) >= 500U))
  {
    lastAutoLedToggleMs = now;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BotonAzul_Pin)
  {
    buttonEdgePending = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint16_t nextHead = (uartRxHead + 1U) & UART_RX_BUFFER_MASK;
    if (nextHead == uartRxTail)
    {
      uartRxOverflowCount++;
      uartRxOverflowPending = 1U;
    }
    else
    {
      uartRxBuffer[uartRxHead] = uartRxByte;
      uartRxHead = nextHead;
    }

    if (HAL_UART_Receive_IT(huart, &uartRxByte, 1U) != HAL_OK)
    {
      uartRxErrorCount++;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uartRxErrorCount++;
    uartRxOverflowPending = 1U;
    if (HAL_UART_Receive_IT(huart, &uartRxByte, 1U) != HAL_OK)
    {
      uartRxErrorCount++;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uartTxBusy = 0U;
    UART_TxStartNext();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    appTickMs++;
  }
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  uint8_t testMessage[] = "USART2 lista\r\n";
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  if (HAL_UART_Receive_IT(&huart2, &uartRxByte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_UART_Transmit(&huart2, testMessage, sizeof(testMessage) - 1U, 100U);

  buttonStableState = HAL_GPIO_ReadPin(BotonAzul_GPIO_Port, BotonAzul_Pin);
  buttonCandidateState = buttonStableState;
  if (buttonStableState == GPIO_PIN_RESET)
  {
    buttonPressStartMs = appTickMs;
  }
  fixedPulse = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
  lastDutyUpdateMs = appTickMs;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = appTickMs;
    UART_ProcessRx();
    UART_TxStartNext();
    Button_Process(now);

    while ((rampMode != 0U) && (paused == 0U) &&
           ((uint32_t)(now - lastDutyUpdateMs) >= 20U))
    {
      lastDutyUpdateMs += 20U;

      if (rampIncreasing != 0U)
      {
        if (dutyCycle < dutyMax)
        {
          dutyCycle++;
        }
        else
        {
          rampIncreasing = 0U;
          dutyCycle--;
        }
      }
      else if (dutyCycle > dutyMin)
      {
        dutyCycle--;
      }
      else
      {
        rampIncreasing = 1U;
        dutyCycle++;
      }

      PWM_ApplyCurrentMode();
      now = appTickMs;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BotonAzul_Pin */
  GPIO_InitStruct.Pin = BotonAzul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BotonAzul_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
