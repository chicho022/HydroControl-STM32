/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { MODE_PID = 0, MODE_GS = 1 } ctrl_mode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PIN   GPIO_PIN_3
#define TRIG_PORT  GPIOA
#define ECHO_PIN   GPIO_PIN_8
#define ECHO_PORT  GPIOA

#define INJ_IN1_PORT GPIOA
#define INJ_IN1_PIN  GPIO_PIN_6
#define INJ_IN2_PORT GPIOA
#define INJ_IN2_PIN  GPIO_PIN_7

#define EXT_IN3_PORT GPIOB
#define EXT_IN3_PIN  GPIO_PIN_0
#define EXT_IN4_PORT GPIOB
#define EXT_IN4_PIN  GPIO_PIN_1

#define INJ_PWM_TIM   (&htim2)
#define INJ_PWM_CH    TIM_CHANNEL_1   // ENA

#define EXT_PWM_TIM   (&htim2)
#define EXT_PWM_CH    TIM_CHANNEL_2   // ENB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

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
/* ===== Control (solo cálculo + telemetría) ===== */
#define CTRL_TS_MS 100

static float Kp_base = 270.0565f;
static float Ki_base = 3.2444f;
static float Kd_base = 0.1013f;

static float I_n = 0.0f;
static float e_prev = 0.0f;

static float dist_f = 0.0f;
static uint8_t dist_f_init = 0;
static const float a_filt = 0.7f;

static const float u_min = -255.0f;
static const float u_max =  255.0f;

static float u_last = 0.0f;

static uint32_t last_ctrl_ms = 0;

static inline float f_abs(float x) { return (x >= 0.0f) ? x : -x; }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void send_line(const char *s);
static void send_status(const char *tag);
static void process_cmd_main(const char *s);
static float control_step(float y_cm, float sp_cm, uint8_t ok, float dt);
static void pumps_write(float u_sat);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
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

static float control_step(float y_cm, float sp_cm, uint8_t ok, float dt)
{
  if (!ok) {
    I_n = 0.0f;
    e_prev = 0.0f;
    u_last = 0.0f;
    return 0.0f;
  }

  float e = y_cm - sp_cm;

  float Kp = Kp_base, Ki = Ki_base, Kd = Kd_base;

  if (g_mode == MODE_GS) {
    float ae = f_abs(e);
    if ((e * e_prev) < 0.0f) { Kp *= 0.5f; Ki *= 0.7f; Kd *= 1.3f; }
    else if (ae > 4.0f)      { Kp *= 1.9f; Ki *= 1.5f; Kd *= 0.9f; }
    else if (ae > 1.5f)      { Kp *= 1.7f; Ki *= 1.2f; Kd *= 0.5f; }
    else if (ae < 0.2f)      { Kp = 0.0f; Ki = 0.0f; Kd = 0.0f; }
    else                     { Kp *= 0.6f; Ki *= 0.9f; Kd *= 1.2f; }
  }

  I_n += e * dt;
  float D = (e - e_prev) / dt;
  float u = Kp * e + Ki * I_n + Kd * D;

  float u_sat = u;
  if (u_sat > u_max) u_sat = u_max;
  if (u_sat < u_min) u_sat = u_min;

  /* anti-windup suave */
  const float Kw = 0.05f;
  I_n += Kw * (u_sat - u) * dt;

  e_prev = e;
  u_last = u_sat;

  return u_sat;
}
static void pumps_write(float u_sat)
{
  float inj = 0.0f, ext = 0.0f;

  if (u_sat > 0.0f) inj = u_sat;       // inyección
  else if (u_sat < 0.0f) ext = -u_sat; // extracción

  if (inj > 255.0f) inj = 255.0f;
  if (ext > 255.0f) ext = 255.0f;

  uint32_t arr_inj = __HAL_TIM_GET_AUTORELOAD(INJ_PWM_TIM);
  uint32_t arr_ext = __HAL_TIM_GET_AUTORELOAD(EXT_PWM_TIM);

  uint32_t duty_inj = (uint32_t)((inj / 255.0f) * (float)arr_inj);
  uint32_t duty_ext = (uint32_t)((ext / 255.0f) * (float)arr_ext);

  __HAL_TIM_SET_COMPARE(INJ_PWM_TIM, INJ_PWM_CH, duty_inj);
  __HAL_TIM_SET_COMPARE(EXT_PWM_TIM, EXT_PWM_CH, duty_ext);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  pumps_write(0.0f);

  /* Sentidos fijos (ajusta si gira al revés) */
  HAL_GPIO_WritePin(INJ_IN1_PORT, INJ_IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(INJ_IN2_PORT, INJ_IN2_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(EXT_IN3_PORT, EXT_IN3_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EXT_IN4_PORT, EXT_IN4_PIN, GPIO_PIN_RESET);

  /* Arranca apagado */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);


  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  send_line("STM32 OK: manda SP,16 o MODE,1 / MODE,0\n");
  send_status("BOOT");
  last_hb_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

     if (now - last_ctrl_ms < CTRL_TS_MS) { HAL_Delay(1); continue; }

     float dt = (float)CTRL_TS_MS / 1000.0f;
     last_ctrl_ms = now;

     /* ===== Ultrasonico ===== */
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

     /* ===== Filtro simple ===== */
     float y = (float)Distance;
     if (!dist_f_init) { dist_f = y; dist_f_init = 1; }
     else dist_f = a_filt * dist_f + (1.0f - a_filt) * y;

     /* ===== Control ===== */
     float u_sat = control_step(dist_f, (float)SP_n, ok, dt);

     /* ===== ACTUACIÓN PWM (NUEVO) ===== */
     pumps_write(u_sat);

     /* ===== Telemetría SIN floats: T,dist,sp,u,mode (x100) ===== */
     int32_t dist_x100 = (int32_t)(dist_f * 100.0f);
     int32_t sp_x100   = (int32_t)(((float)SP_n) * 100.0f);
     int32_t u_x100    = (int32_t)(u_sat * 100.0f);
     int32_t mode_i    = (int32_t)g_mode;

     char msg[120];
     int n = snprintf(msg, sizeof(msg), "T,%ld,%ld,%ld,%ld\n",
                      (long)dist_x100, (long)sp_x100, (long)u_x100, (long)mode_i);
     HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)n, 100);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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