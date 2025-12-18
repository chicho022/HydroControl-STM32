/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define TRIG_PIN   GPIO_PIN_9
#define TRIG_PORT  GPIOA
#define ECHO_PIN   GPIO_PIN_8
#define ECHO_PORT  GPIOA
/* USER CODE END PD */

/* USER CODE BEGIN PTD */
typedef enum { MODE_PID = 0, MODE_GS = 1 } ctrl_mode_t;
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;

/* RX por interrupción */
static uint8_t  rx_byte;
static char     rx_line[64];
static uint16_t rx_idx = 0;

/* comando listo (parseo en main) */
static volatile uint8_t cmd_ready = 0;
static char cmd_buf[64];

/* Estado */
static volatile float SP_n = 16.0f;
static volatile uint8_t sp_changed = 0;

static volatile ctrl_mode_t g_mode = MODE_PID;
static volatile uint8_t mode_changed = 0;

static uint32_t last_hb_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
static void send_line(const char *s);
static void send_status(const char *tag);
static void process_cmd_main(const char *s);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void send_line(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

/* Estado compacto sin floats (x100) */
static void send_status(const char *tag)
{
  int32_t sp_x100 = (int32_t)((float)SP_n * 100.0f);
  int32_t mode_i  = (int32_t)g_mode;

  char msg[96];
  int n = snprintf(msg, sizeof(msg), "%s,SP=%ld,MODE=%ld\n",
                   tag, (long)sp_x100, (long)mode_i);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)n, 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) return;

  char c = (char)rx_byte;

  if (c == '\n' || c == '\r') {
    if (rx_idx > 0) {
      rx_line[rx_idx] = 0;

      if (!cmd_ready) {
        strncpy(cmd_buf, rx_line, sizeof(cmd_buf) - 1);
        cmd_buf[sizeof(cmd_buf) - 1] = 0;
        cmd_ready = 1;
      }
      rx_idx = 0;
    }
  } else {
    if (rx_idx < sizeof(rx_line) - 1) rx_line[rx_idx++] = c;
    else rx_idx = 0;
  }

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

static void process_cmd_main(const char *s)
{
  /* Eco RX para debug (como tu prueba) */
  char rxm[96];
  int rn = snprintf(rxm, sizeof(rxm), "RX,%s\n", s);
  HAL_UART_Transmit(&huart1, (uint8_t*)rxm, (uint16_t)rn, 100);

  /* SP */
  if (strncmp(s, "SP,", 3) == 0) {
    char *endp = NULL;
    float v = strtof(s + 3, &endp);

    if (endp != (s + 3)) {
      SP_n = v;
      sp_changed = 1;

      int32_t v_x100 = (int32_t)(v * 100.0f);
      char ack[64];
      int n = snprintf(ack, sizeof(ack), "ACK_SP,%ld\n", (long)v_x100);
      HAL_UART_Transmit(&huart1, (uint8_t*)ack, (uint16_t)n, 100);

      send_status("SP_SET");
    } else {
      send_line("ERR_SP\n");
    }
    return;
  }

  /* MODE */
  if (strncmp(s, "MODE,", 5) == 0) {
    int m = atoi(s + 5);
    g_mode = (m == 1) ? MODE_GS : MODE_PID;
    mode_changed = 1;

    char ack[64];
    int n = snprintf(ack, sizeof(ack), "ACK_MODE,%d\n", (g_mode == MODE_GS) ? 1 : 0);
    HAL_UART_Transmit(&huart1, (uint8_t*)ack, (uint16_t)n, 100);

    send_status("MODE_SET");
    return;
  }

  /* desconocido */
  char unk[96];
  int n = snprintf(unk, sizeof(unk), "ACK,%s\n", s);
  HAL_UART_Transmit(&huart1, (uint8_t*)unk, (uint16_t)n, 100);
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  send_line("STM32 OK: manda SP,16 o MODE,1 / MODE,0\n");
  send_status("BOOT");
  last_hb_ms = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    if (cmd_ready) {
      __disable_irq();
      cmd_ready = 0;
      char local[64];
      strncpy(local, cmd_buf, sizeof(local) - 1);
      local[sizeof(local) - 1] = 0;
      __enable_irq();

      process_cmd_main(local);
    }

    uint32_t now = HAL_GetTick();

    if (now - last_hb_ms >= 1000) {
      last_hb_ms = now;
      send_status("HB");
    }

    if (sp_changed) {
      sp_changed = 0;
      send_status("SP_NOW");
    }

    if (mode_changed) {
      mode_changed = 0;
      send_status("MODE_NOW");
    }

    /* Ultrasonico + telemetria simple (igual que antes) */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 10) {}
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    uint8_t ok = 1;

    pMillis = HAL_GetTick();
    while (!HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) && (HAL_GetTick() - pMillis) < 10) {}
    if (!HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) ok = 0;
    Value1 = __HAL_TIM_GET_COUNTER(&htim1);

    pMillis = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) && (HAL_GetTick() - pMillis) < 50) {}
    if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) ok = 0;
    Value2 = __HAL_TIM_GET_COUNTER(&htim1);

    if (!ok || (Value2 <= Value1)) Distance = 0;
    else Distance = (uint16_t)((Value2 - Value1) * 0.034f / 2.0f);

    int32_t sp_x100 = (int32_t)((float)SP_n * 100.0f);
    int32_t mode_i  = (int32_t)g_mode;

    char msg[96];
    int n = snprintf(msg, sizeof(msg), "D,%u,SP=%ld,MODE=%ld\n",
                     Distance, (long)sp_x100, (long)mode_i);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)n, 100);

    HAL_Delay(500);
  }
}

/* Deja el resto (SystemClock_Config, MX_*, Error_Handler) como CubeMX */
/* ======== CubeMX generated init functions (como los tuyos) ======== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif