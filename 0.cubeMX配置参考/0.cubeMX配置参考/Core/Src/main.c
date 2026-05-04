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
#include "led.h"
#include "oled.h"
#include "pid.h"
#include "usart.h"
#include "vofa.h"
#include "adc.h"
#include "motor.h"
#include "crsf_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART1_ADC_DEBUG_ENABLE        0U
#define UART1_ADC_DEBUG_PERIOD_MS   100U

#define CURRENT_PID_LOOP_PERIOD_MS   20U
#define CURRENT_PID_OUTPUT_LIMIT_MA  5000.0f
#define CURRENT_PID_INT_LIMIT        1000.0f
#define CURRENT_PID_FEEDBACK_CH      1U  /* 1 = ADC ch1, 2 = ADC ch2 */
#define VOFA_TX_PERIOD_MS            10U
#define VOFA_TX_FLOAT_COUNT          4U
/* CRSF UART link on the FC side uses 420000 8N1 in this project. */
#define CRSF_UART_BAUDRATE        420000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
static volatile float s_current_pid_fwd_kp = 0.90f;
static volatile float s_current_pid_fwd_ki = 0.35f;
static volatile float s_current_pid_fwd_kd = 0.00f;
static volatile float s_current_pid_rev_kp = 0.90f;
static volatile float s_current_pid_rev_ki = 0.35f;
static volatile float s_current_pid_rev_kd = 0.00f;
static volatile float s_current_pid_target_mA = 0.0f;
static volatile float s_motor_b_target_mA     = 0.0f;
static volatile float s_current_pid_feedback_mA = 0.0f;
static volatile float s_current_pid_error_mA = 0.0f;
static volatile float s_current_pid_output_mA = 0.0f;
static volatile uint8_t s_current_pid_param_updated = 0U;
static volatile uint8_t s_target_from_vofa = 1U;
static current_pid_t s_current_pid = {0};
static uint32_t s_reset_flags = 0U;
volatile uint8_t g_pid_run_flag = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_CaptureResetCause(void);
static void App_PrintResetCause(void);
static void App_CurrentPidApplyConfig(void);
static float App_CurrentPidStep(float target_mA, float feedback_mA, float dt_s);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_CaptureResetCause(void)
{
  s_reset_flags = RCC->RSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

static void App_PrintResetCause(void)
{
  printf("RESET: RSR=0x%08lX [%s%s%s%s%s%s]\r\n",
      (unsigned long)s_reset_flags,
      ((s_reset_flags & RCC_RSR_PINRSTF) != 0U) ? "PIN " : "",
      ((s_reset_flags & RCC_RSR_PORRSTF) != 0U) ? "POR " : "",
      ((s_reset_flags & RCC_RSR_BORRSTF) != 0U) ? "BOR " : "",
      ((s_reset_flags & RCC_RSR_SFTRSTF) != 0U) ? "SFT " : "",
      ((s_reset_flags & RCC_RSR_IWDG1RSTF) != 0U) ? "IWDG " : "",
      ((s_reset_flags & RCC_RSR_WWDG1RSTF) != 0U) ? "WWDG " : "");
}

static void App_CurrentPidApplyConfig(void)
{
  CurrentPID_Init(&s_current_pid,
                  s_current_pid_fwd_kp,
                  s_current_pid_fwd_ki,
                  s_current_pid_fwd_kd,
                  s_current_pid_rev_kp,
                  s_current_pid_rev_ki,
                  s_current_pid_rev_kd,
                  CURRENT_PID_INT_LIMIT,
                  CURRENT_PID_OUTPUT_LIMIT_MA);
}

static float App_CurrentPidStep(float target_mA, float feedback_mA, float dt_s)
{
  s_current_pid_error_mA = target_mA - feedback_mA;
  return CurrentPID_Compute(&s_current_pid, target_mA, feedback_mA, dt_s);
}

static void App_VofaParamHandler(uint16_t id, float value)
{
  uint8_t handled = 0U;

	if (id == VOFA_PARAM_ID_CURRENT_FWD_KP)
	{
    s_current_pid_fwd_kp = value;
    handled = 1U;
	}
	else if (id == VOFA_PARAM_ID_CURRENT_FWD_KI)
	{
    s_current_pid_fwd_ki = value;
    handled = 1U;
	}
	else if (id == VOFA_PARAM_ID_CURRENT_REV_KP)
	{
    s_current_pid_rev_kp = value;
    handled = 1U;
	}
	else if (id == VOFA_PARAM_ID_CURRENT_REV_KI)
	{
    s_current_pid_rev_ki = value;
    handled = 1U;
	}
	else if (id == VOFA_PARAM_ID_TARGET_MA)
	{
    s_current_pid_target_mA = value;
    s_target_from_vofa = 1U;
  }

  if (handled != 0U)
  {
    s_current_pid_param_updated = 1U;
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
  /* Test variables for UART printf retargeting */
//  uint16_t a = 128;
//	float b = 9.123456; 
//	char* show_buffer[128];
	uint16_t adc_raw[2] = {0};
  uint32_t last_uart1_adc_tick = 0U;
  uint32_t last_uart1_rc_tick = 0U;
  int16_t last_rc_lx10 = 0;
  int16_t last_rc_ly10 = 0;
  int16_t last_rc_rx10 = 0;
  int16_t last_rc_ry10 = 0;
  uint8_t last_rc_a = 0U;
  uint8_t last_rc_b = 0U;
  uint8_t last_rc_c = 0U;
  uint8_t last_rc_d = 0U;
  uint8_t last_rc_e = 0U;
  uint8_t last_rc_f = 0U;
  uint8_t last_rc_lq = 0U;
  uint16_t last_rc_hb = 0U;
  uint8_t rc_print_initialized = 0U;
  uint32_t last_vofa_tx_tick = 0U;
  uint32_t last_vofa_err_print_tick = 0U;
  uint32_t last_vofa_err_count = 0U;
  uint32_t last_crsf_diag_tick = 0U;
  uint32_t last_crsf_no_frame_tick = 0U;
  uint8_t last_crsf_connected = 0xFFU;
  uint32_t last_crsf_frame_count = 0U;
  uint32_t last_crsf_err_count = 0U;
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_Init();
  App_PrintResetCause();
  printf("BOOT: LED_Init done\r\n");

  VOFA_Init();
  printf("BOOT: VOFA_Init done\r\n");

  VOFA_SetParamCallback(App_VofaParamHandler);

  printf("BOOT: VOFA_Begin start\r\n");
  if (VOFA_Begin() != HAL_OK)
	{
		Error_Handler();
	}
  printf("BOOT: VOFA_Begin done\r\n");
  App_CurrentPidApplyConfig();

  // printf("BOOT: OLED_Init start\r\n");
	// OLED_Init();
  // printf("BOOT: OLED_Init done\r\n");

	/* Initialize ADC for current sensing */
  printf("BOOT: ADC_Init start\r\n");
	ADC_Init();
  printf("BOOT: ADC_Init done\r\n");

  printf("BOOT: ADC_StartDMA start\r\n");
	if (ADC_StartDMA() != HAL_OK)
	{
		Error_Handler();
	}
  printf("BOOT: ADC_StartDMA done\r\n");

  /* Initialize TIM1/TIM2 and start PWM channels.
   * Starting TIM1 also enables TIM1 TRGO, which triggers ADC conversions. */
  Motor_Init();
  printf("BOOT: Motor_Init done\r\n");

  CRSF_Init();
  printf("BOOT: CRSF_Init done (USART3 PB11 @ %lu)\r\n", (unsigned long)CRSF_UART_BAUDRATE);

  printf("VOFA param map: #P1=fwd_kp #P2=fwd_ki #P3=rev_kp #P4=rev_ki #P5=target_mA\r\n");
  printf("VOFA wave map: ch0=target ch1=feedback ch2=output ch3=error (4ch, 10Hz)\r\n");
  
  printf("STM32H743 initialized\r\n");
	printf("printf testing\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// LED1_Toggle;
		// HAL_Delay(1);

    VOFA_ProcessRx();

    if ((HAL_GetTick() - last_vofa_tx_tick) >= VOFA_TX_PERIOD_MS)
    {
      float vofa_frame[VOFA_TX_FLOAT_COUNT];

      last_vofa_tx_tick = HAL_GetTick();

      vofa_frame[0] = s_current_pid_target_mA;
      vofa_frame[1] = s_current_pid_feedback_mA;
      vofa_frame[2] = s_current_pid_output_mA;
      vofa_frame[3] = s_current_pid_error_mA;

      (void)VOFA_SendFloats(vofa_frame, VOFA_TX_FLOAT_COUNT);
    }

    if ((HAL_GetTick() - last_vofa_err_print_tick) >= 200U)
    {
      uint32_t err_count = VOFA_GetUartErrorCount();
      if (err_count != last_vofa_err_count)
      {
        last_vofa_err_count = err_count;
        printf("VOFA UART err cnt=%lu code=0x%08lX isr=0x%08lX\r\n",
            (unsigned long)err_count,
            (unsigned long)VOFA_GetUartLastError(),
            (unsigned long)VOFA_GetUartLastIsr());
      }
      last_vofa_err_print_tick = HAL_GetTick();
    }

    if ((HAL_GetTick() - last_crsf_diag_tick) >= 200U)
    {
      crsf_status_t st = {0};
      uint32_t crsf_idle_ms;
      CRSF_GetStatus(&st);
      crsf_idle_ms = HAL_GetTick() - st.last_frame_tick;

      if ((st.connected != last_crsf_connected)
          || (st.frame_count != last_crsf_frame_count)
          || (st.error_count != last_crsf_err_count))
      {
        last_crsf_connected = st.connected;
        last_crsf_frame_count = st.frame_count;
        last_crsf_err_count = st.error_count;
        printf("CRSF st: conn=%u idle_ms=%lu frame=%u err=%u parse=%u uart_tot=%u uart(pe=%u fe=%u ne=%u ore=%u) last=0x%08lX\r\n",
            (unsigned int)st.connected,
            (unsigned long)crsf_idle_ms,
            (unsigned int)st.frame_count,
            (unsigned int)st.error_count,
          (unsigned int)st.parse_error_count,
          (unsigned int)st.uart_error_total,
            (unsigned int)st.uart_err_pe,
            (unsigned int)st.uart_err_fe,
            (unsigned int)st.uart_err_ne,
            (unsigned int)st.uart_err_ore,
            (unsigned long)st.last_uart_error);
        printf("  parse_detail: sync_loss=%u sync_skip=%u len_err=%u crc=%u unknown_type=%u rc_fail=%u short=%u\r\n",
          (unsigned int)st.parse_detail.sync_loss_events,
          (unsigned int)st.parse_detail.sync_skip_bytes,
            (unsigned int)st.parse_detail.len_invalid,
            (unsigned int)st.parse_detail.crc_error,
            (unsigned int)st.parse_detail.unknown_type,
            (unsigned int)st.parse_detail.rc_decode_fail,
            (unsigned int)st.parse_detail.frame_too_short);
      }
      last_crsf_diag_tick = HAL_GetTick();
      
      if ((st.frame_count == 0U) && ((HAL_GetTick() - last_crsf_no_frame_tick) >= 1000U))
      {
        last_crsf_no_frame_tick = HAL_GetTick();
        printf("CRSF no frame: conn=%u idle_ms=%lu uart_tot=%u last=0x%08lX\r\n",
            (unsigned int)st.connected,
            (unsigned long)crsf_idle_ms,
            (unsigned int)st.uart_error_total,
            (unsigned long)st.last_uart_error);
      }
    }

    if ((HAL_GetTick() - last_uart1_rc_tick) >= 100U)
    {
      ELRS_Data rc_data = {0};
      int16_t lx10;
      int16_t ly10;
      int16_t rx10;
      int16_t ry10;
      CRSF_GetElrsData(&rc_data);
      if (rc_data.valid != 0U)
      {
        lx10 = (int16_t)(rc_data.Left_X * 10.0f);
        ly10 = (int16_t)(rc_data.Left_Y * 10.0f);
        rx10 = (int16_t)(rc_data.Right_X * 10.0f);
        ry10 = (int16_t)(rc_data.Right_Y * 10.0f);

        if ((rc_print_initialized == 0U)
            || (lx10 != last_rc_lx10)
            || (ly10 != last_rc_ly10)
            || (rx10 != last_rc_rx10)
            || (ry10 != last_rc_ry10)
            || (rc_data.A != last_rc_a)
            || (rc_data.B != last_rc_b)
            || (rc_data.C != last_rc_c)
            || (rc_data.D != last_rc_d)
            || (rc_data.E != last_rc_e)
            || (rc_data.F != last_rc_f)
            || (rc_data.uplink_Link_quality != last_rc_lq)
            || (rc_data.heartbeat_counter != last_rc_hb))
        {
          printf("RC LX=%.1f LY=%.1f RX=%.1f RY=%.1f A=%u B=%u C=%u D=%u E=%u F=%u LQ=%u HB=%u\r\n",
              (double)rc_data.Left_X,
              (double)rc_data.Left_Y,
              (double)rc_data.Right_X,
              (double)rc_data.Right_Y,
              (unsigned int)rc_data.A,
              (unsigned int)rc_data.B,
              (unsigned int)rc_data.C,
              (unsigned int)rc_data.D,
              (unsigned int)rc_data.E,
              (unsigned int)rc_data.F,
              (unsigned int)rc_data.uplink_Link_quality,
              (unsigned int)rc_data.heartbeat_counter);

          last_rc_lx10 = lx10;
          last_rc_ly10 = ly10;
          last_rc_rx10 = rx10;
          last_rc_ry10 = ry10;
          last_rc_a = rc_data.A;
          last_rc_b = rc_data.B;
          last_rc_c = rc_data.C;
          last_rc_d = rc_data.D;
          last_rc_e = rc_data.E;
          last_rc_f = rc_data.F;
          last_rc_lq = rc_data.uplink_Link_quality;
          last_rc_hb = rc_data.heartbeat_counter;
          rc_print_initialized = 1U;
        }
      }
      last_uart1_rc_tick = HAL_GetTick();
    }


    if (g_pid_run_flag != 0U)
		{
			float current_ch1_mA;
			float current_ch2_mA;

      g_pid_run_flag = 0U;

      if (s_current_pid_param_updated != 0U)
      {
        s_current_pid_param_updated = 0U;
        App_CurrentPidApplyConfig();
        printf("VOFA fwd(kp=%.3f ki=%.3f kd=%.3f) rev(kp=%.3f ki=%.3f kd=%.3f) target=%.3f\r\n",
            (double)s_current_pid_fwd_kp,
            (double)s_current_pid_fwd_ki,
            (double)s_current_pid_fwd_kd,
            (double)s_current_pid_rev_kp,
            (double)s_current_pid_rev_ki,
            (double)s_current_pid_rev_kd,
            (double)s_current_pid_target_mA);
      }

      /* CRSF timeout check + RC→target mixing */
      CRSF_Process();
      {
          crsf_input_t rc = {0};
          CRSF_GetInput(&rc);
          if (rc.valid && (rc.estop == 0U) && (rc.handbrake == 0U))
          {
              /* Tank-steer mixing: A = throttle+steer, B = throttle-steer */
              s_current_pid_target_mA = (rc.throttle + rc.steering) * CURRENT_PID_OUTPUT_LIMIT_MA;
              s_motor_b_target_mA     = (rc.throttle - rc.steering) * CURRENT_PID_OUTPUT_LIMIT_MA;
              s_target_from_vofa      = 0U;
          }
          else
          {
              if (s_target_from_vofa == 0U)
              {
                  s_current_pid_target_mA = 0.0f;
              }
              s_motor_b_target_mA = 0.0f;
          }
      }
			
			/* Read ADC current values */
			ADC_GetValues(adc_raw);
			current_ch1_mA = ADC_RawToCurrent_mA(adc_raw[0]);
			current_ch2_mA = ADC_RawToCurrent_mA(adc_raw[1]);

#if (UART1_ADC_DEBUG_ENABLE != 0U)
      if ((HAL_GetTick() - last_uart1_adc_tick) >= UART1_ADC_DEBUG_PERIOD_MS)
      {
        last_uart1_adc_tick = HAL_GetTick();
    printf("ADC raw: ch1=%u, %.2fmA | ch2=%u, %.2fmA | fb_ch=%u, fb=%.2fmA\r\n",
            (unsigned int)adc_raw[0],
            (double)current_ch1_mA,
            (unsigned int)adc_raw[1],
    (double)current_ch2_mA,
    (unsigned int)CURRENT_PID_FEEDBACK_CH,
    (double)s_current_pid_feedback_mA);
      }
#endif

#if (CURRENT_PID_FEEDBACK_CH == 1U)
      s_current_pid_feedback_mA = current_ch1_mA;
#else
      s_current_pid_feedback_mA = current_ch2_mA;
#endif

      s_current_pid_output_mA = App_CurrentPidStep(
          s_current_pid_target_mA,
          s_current_pid_feedback_mA,
          (float)CURRENT_PID_LOOP_PERIOD_MS / 1000.0f);

      Motor_SetOutput(MOTOR_A, s_current_pid_output_mA, CURRENT_PID_OUTPUT_LIMIT_MA);
      Motor_SetOutput(MOTOR_B, s_motor_b_target_mA,    CURRENT_PID_OUTPUT_LIMIT_MA);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x307075B1;
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
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 12000-1;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 12000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 420000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
		LED1_Toggle;
		HAL_Delay(200);
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
