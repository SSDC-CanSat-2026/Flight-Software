/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "app_fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Inc/config.h"
#include "../Inc/global.h"
#include "../Inc/microSD.h"
#include "../Inc/FreeRTOSConfig.h"
#include "../../Drivers/MS5607/MS5607SPI.h"       // Pressure and Temperature Sensor
#include "../../Drivers/ICM42688P/ICM42688PSPI.h" // Accelerometer and Gyro Sensor
#include "../../Drivers/STUSB4500LBJR/USB_port.h" // USB PD controller
#include "../../Drivers/LC76G/LC76G.h"         // GPS Module
#include "../../Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE 128
#define XBEE_MAX_PAYLOAD 80   // Safe value

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CORDIC_HandleTypeDef hcordic;

FMAC_HandleTypeDef hfmac;

I2C_HandleTypeDef hi2c3;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId readSensorsHandle;
osThreadId readCommandsHandle;
osThreadId sendTelemetryHandle;
osThreadId guideNavCtrlHandle;
osThreadId InitHandle;
osSemaphoreId globalDataHandle;
/* USER CODE BEGIN PV */

uint16_t Timer1, Timer2;
uint8_t dma_buffer[BUFFER_SIZE]   = { 0 };
char transmit_buffer[BUFFER_SIZE] = { 0 };
char receive_buffer[BUFFER_SIZE]  = { 0 };
char command_buffer[BUFFER_SIZE]  = { 0 };

// Flags for GPS and XBEE since they use UART DMA
volatile uint16_t GPS_SIZE 	   = 0;
volatile uint8_t GPS_READY 	   = 0;
volatile uint16_t COMMAND_SIZE = 0;
volatile uint8_t COMMAND_READY = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FMAC_Init(void);
static void MX_I2C3_Init(void);
static void MX_RNG_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM1_Init(void);
void StartReadSensors(void const * argument);
void StartReadCommands(void const * argument);
void StartSendTelemetry(void const * argument);
void StartGNC(void const * argument);
void StartInit(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == UART5)
  {
    if (!GPS_READY)
    {
		GPS_SIZE = size;
		GPS_READY = 1;
		memcpy(receive_buffer, dma_buffer, size);
		memset(dma_buffer, 0, size);
    }
  }
  else if (huart->Instance == USART3)
  {
    if (!COMMAND_READY)
    {
		COMMAND_SIZE = size;
		COMMAND_READY = 1;
		memcpy(command_buffer, dma_buffer, size);
    }
  }
  else
  {
    // FIXME : Change this for a DBG LED in the new code
	  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
  }

  HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_buffer, BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_ErrorCallback(UART_HandleTypeDef *huart) {
	while(1) {
		// 1 quick 2 slow to show a UART error
		HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
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
  init_mission_data();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  MX_I2C3_Init();
  MX_RNG_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Disable ALL chip selects
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMP_nCS_GPIO_Port, BMP_nCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SD_nCS_GPIO_Port, SD_nCS_Pin, GPIO_PIN_SET);

  // Hold GPS in reset (LOW)
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);

  // Fully reset UART5 peripheral
  __HAL_RCC_UART5_FORCE_RESET();
  __HAL_RCC_UART5_RELEASE_RESET();
  MX_UART5_Init();   // reinitialize UART

  // Clear all flags
  __HAL_UART_CLEAR_OREFLAG(&huart5);
  __HAL_UART_CLEAR_FEFLAG(&huart5);
  __HAL_UART_CLEAR_NEFLAG(&huart5);
  __HAL_UART_CLEAR_PEFLAG(&huart5);

  // Fully reset UART3 peripheral
  __HAL_RCC_USART3_FORCE_RESET();
  __HAL_RCC_USART3_RELEASE_RESET();
  MX_USART3_UART_Init();   // reinitialize UART

  // Clear all flags
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_FEFLAG(&huart3);
  __HAL_UART_CLEAR_NEFLAG(&huart3);
  __HAL_UART_CLEAR_PEFLAG(&huart3);

  // Now release GPS reset
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(5000);

  teseo_INIT(&huart5);


  // Enable DMA call backs
  // UART 5
  // Check if ORE flag is set, which can happen if data is present on UART RX line
  if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
  }
  // receive until idle, then trigger interrupt
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dma_buffer, BUFFER_SIZE); // receive until idle, then trigger interrupt
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT); // Disables "Half Transfer" interrupt

  // USART 3
  // Check if ORE flag is set, which can happen if data is present on UART RX line
//  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) {
//    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF);
//  }
//  // receive until idle, then trigger interrupt
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_buffer, BUFFER_SIZE); // receive until idle, then trigger interrupt
//  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // Disables "Half Transfer" interrupt

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of globalData */
  osSemaphoreDef(globalData);
  globalDataHandle = osSemaphoreCreate(osSemaphore(globalData), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of readSensors */
  osThreadDef(readSensors, StartReadSensors, osPriorityNormal, 0, 512);
  readSensorsHandle = osThreadCreate(osThread(readSensors), NULL);

  /* definition and creation of readCommands */
  osThreadDef(readCommands, StartReadCommands, osPriorityNormal, 0, 512);
  readCommandsHandle = osThreadCreate(osThread(readCommands), NULL);

  /* definition and creation of sendTelemetry */
  osThreadDef(sendTelemetry, StartSendTelemetry, osPriorityNormal, 0, 512);
  sendTelemetryHandle = osThreadCreate(osThread(sendTelemetry), NULL);

  /* definition and creation of guideNavCtrl */
  osThreadDef(guideNavCtrl, StartGNC, osPriorityNormal, 0, 512);
  guideNavCtrlHandle = osThreadCreate(osThread(guideNavCtrl), NULL);

  /* definition and creation of Init */
  osThreadDef(Init, StartInit, osPriorityHigh, 0, 512);
  InitHandle = osThreadCreate(osThread(Init), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_LSE;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00C12166;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the reference Clock input
  */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  htim3.Init.Prescaler = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STEP_DIR0_Pin|DEBUG_0_Pin|DEBUG_1_Pin|CAM0_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DEBUG_2_Pin|DRV_DIR0_Pin|CAM1_CTRL_Pin|GPS_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU_nCS_Pin|SD_nCS_Pin|BMP_nCS_Pin|GPS_RST_Pin
                          |USR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP_DIR0_Pin DEBUG_0_Pin DEBUG_1_Pin CAM0_CTRL_Pin */
  GPIO_InitStruct.Pin = STEP_DIR0_Pin|DEBUG_0_Pin|DEBUG_1_Pin|CAM0_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_2_Pin DRV_DIR0_Pin CAM1_CTRL_Pin GPS_WAKE_Pin */
  GPIO_InitStruct.Pin = DEBUG_2_Pin|DRV_DIR0_Pin|CAM1_CTRL_Pin|GPS_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_32k_Pin XBEE_RST_Pin */
  GPIO_InitStruct.Pin = CLK_32k_Pin|XBEE_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_nCS_Pin SD_nCS_Pin BMP_nCS_Pin GPS_RST_Pin
                           USR_LED_Pin */
  GPIO_InitStruct.Pin = IMU_nCS_Pin|SD_nCS_Pin|BMP_nCS_Pin|GPS_RST_Pin
                          |USR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void vApplicationTickHook(void){
	if(Timer1 > 0)
		Timer1--;
	if(Timer2 > 0)
		Timer2--;
}

// This is a call back in case a thread has a stack overflow.
// pcTaskName is the name of the offending task
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    // Forces a breakpoint in the debugger
    __BKPT(0);
    for (;;);
}

// Incase there is an error with a Malloc somewhere
void vApplicationMallocFailedHook(void)
{
    // Fires if pvPortMalloc fails — useful to catch heap exhaustion

	// Forces a breakpoint in the debugger
    __BKPT(0);
    for (;;);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadSensors */
/**
 * @brief  Function implementing the read_sensors thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartReadSensors */
void StartReadSensors(void const * argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
  osStatus stat = osErrorOS;
  /* Infinite loop */
  for (;;)
  {
	stat = osSemaphoreWait(globalDataHandle, 100);
	if (stat != osOK) {
	  osThreadYield();
	  continue;
	}
    // #1 PRIORITY: make sure mutex unlocks no matter what!!!!

    // -> if a HIGHER priority task attempts to access a locked resource,
    // the LOCKING thread assumes the priority of the resource trying to
    // take it

    /*
     * global_mission_data.MODE, global_mission_data.CMD_ECHO,
     * and global_mission_data.PACKET_COUNT
     * is dealt with in readCommands.
     */

//    MS5607Readings MS5607_Data = MS5607ReadValues();
//    if (global_mission_data.MODE == 'F')
//    { // In Flight Mode
//      global_mission_data.PRESSURE = MS5607_Data.pressure_kPa;
//    }
//    else
//    { // In Simulation Mode and need to read from the CSV instead.
//      // TODO
//    }
//    global_mission_data.TEMPERATURE = MS5607_Data.temperature_C;

//    global_mission_data.ALTITUDE = calculateAltitude(global_mission_data.PRESSURE);
//    determineState(global_mission_data.ALTITUDE);

//    ICM42688P_AccelData ICM42688P_Data = ICM42688P_read_data();
//    global_mission_data.GYRO_R = ICM42688P_Data.gyro_r;
//    global_mission_data.GYRO_P = ICM42688P_Data.gyro_p;
//    global_mission_data.GYRO_Y = ICM42688P_Data.gyro_y;

//    global_mission_data.ACCEL_R = ICM42688P_Data.accel_r;
//    global_mission_data.ACCEL_P = ICM42688P_Data.accel_p;
//    global_mission_data.ACCEL_Y = ICM42688P_Data.accel_y;

//    LC76G_gps_data* gps_data = LC76G_read_data(&huart5);
//    global_mission_data.GPS_LATITUDE = gps_data->lat;
//    global_mission_data.GPS_LONGITUDE = gps_data->lon;
//    global_mission_data.GPS_ALTITUDE = gps_data->altitude;
//    global_mission_data.GPS_SATS = gps_data->num_sat_used;

//    snprintf(global_mission_data.GPS_TIME, 9, "%02d:%02d:%02d",
//             gps_data->time_H, gps_data->time_M, gps_data->time_S);

//    RTC_TimeTypeDef sTime = {0};
//    // Needed to unlock time registers
//    RTC_DateTypeDef sDate = {0};
//
//    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    // Needed to unlock time registers
//    if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
//    {
//      Error_Handler();
//    }

	snprintf(global_mission_data.MISSION_TIME, 9, "XX:XX:XX");
//    snprintf(global_mission_data.MISSION_TIME, 9, "%02d:%02d:%02d",
//             sTime.Hours, sTime.Minutes, sTime.Seconds);

    /*
     * Get Voltage and Current from Magnetometer
     */

    /*
     * Get GPS information from GPS
     */

    // Relinquish access to the global_mission_data struct
    osSemaphoreRelease(globalDataHandle);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReadCommands */
/**
* @brief Function implementing the readCommands thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadCommands */
void StartReadCommands(void const * argument)
{
  /* USER CODE BEGIN StartReadCommands */
  for (;;)
  {
    // do interrupts have to be enabled for this? they are in the previous project

    // i honestly dk if this is peak performance tbh
    // something tells me we could just have a char array to begin with but i would wanna
    // wait until we can test to make changes for sure
    uint8_t command_buffer[CMD_BUFFER_LEN];
    HAL_UART_Receive_IT(&huart3, command_buffer, CMD_BUFFER_LEN);

    char *char_arr = (char *)command_buffer;
    char rx_string[CMD_BUFFER_LEN];

    int8_t num_tokens = osSemaphoreWait(globalDataHandle, 100);
	if (num_tokens <= 0) {
		continue; // Semaphore is either unavailable or inputs are wrong
	}

    strncpy(rx_string, char_arr, CMD_BUFFER_LEN);
    if (strncmp(rx_string, "CMD,3174,CX,ON", 14) == 0)
    {
      // set command echo in the global mission struct
      char c_echo[] = "CXON";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // CX OFF command -> stop transmitting telemetry packets
    else if (strncmp(rx_string, "CMD,3174,CX,OFF", 15) == 0)
    {
      // set command echo
      char c_echo[] = "CXOFF";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // ST command -> set mission time
    else if (strncmp(rx_string, "CMD,3174,ST,", 12) == 0)
    {
      // parse the timestamp to set to
      char arg[9];
      char *time_str = rx_string + 12;
      strncpy(arg, time_str, 9);

      // removed this code because GPS is screwed

      // set command echo
      char c_echo[] = "ST";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // SIM ENABLE command -> allow simulation mode to be activated
    else if (strncmp(rx_string, "CMD,3174,SIM,ENABLE", 19) == 0)
    {
      // set command echo
      char c_echo[] = "SIMENABLE";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // SIM ACTIVATE command -> turn simulation mode on
    else if (strncmp(rx_string, "CMD,3174,SIM,ACTIVATE", 21) == 0)
    {
      // check that simulation mode has been activated
      if (simulation_pre == 1)
      {
        // make first simulated pressure value match actual value
        simulated_pressure = global_mission_data.PRESSURE;
        // set command echo
        char c_echo[] = "SIMACT";
        strcpy(global_mission_data.CMD_ECHO, c_echo);
      }
    }
    // SIM DISABLE command -> turn simulation mode off
    else if (strncmp(rx_string, "CMD,3174,SIM,DISABLE", 20) == 0)
    {
      // set command echo
      char c_echo[] = "SIMDIS";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // SIMP command -> add simulated pressure data
    else if (strncmp(rx_string, "CMD,3174,SIMP,", 14) == 0)
    {
      // parse inputed pressure data
//      char *pressure_str = rx_string + 14;
//      char *str_end;
      long pressure_pa = atof(rx_string + 14);
      // if (str_end == pressure_str || *str_end != '\0')
      // it wasn't a valid number
      // set simulated pressure to parsed value
      simulated_pressure = pressure_pa;

      // set command echo
      char c_echo[] = "SIMP";
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // CAL command -> calibrate altitude
    else if (strncmp(rx_string, "CMD,3174,CAL", 12) == 0)
    {
      // set command echo
      char c_echo[] = "CAL";
      char new_state[]= "LAUNCH_PAD";

      calibrateAltitudeHistory();

      strcpy(global_mission_data.STATE, new_state);
      strcpy(global_mission_data.CMD_ECHO, c_echo);
    }
    // MEC WIRE ON command -> actuate (servos?)
    else if (strncmp(rx_string, "CMD,3174,MEC,WIRE,ON", 20) == 0)
    {
      // activate MEC command
    }
    // MEC WIRE OFF command -> stop actuations
    else if (strncmp(rx_string, "CMD,3174,MEC,WIRE,OFF", 21) == 0)
    {
      // turn off MEC command (servos for GNC?)
    }

    osSemaphoreRelease(globalDataHandle);

    // clear command buffer
    memset(rx_string, 0, sizeof(rx_string)); // Can someone double check if this is supposed to clear the command buffer?
  }
  /* USER CODE END StartReadCommands */
}

/* USER CODE BEGIN Header_StartSendTelemetry */
/**
 * @brief Function implementing the send_telemetry thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSendTelemetry */
void StartSendTelemetry(void const * argument)
{
  /* USER CODE BEGIN StartSendTelemetry */
	osStatus stat = osErrorOS;
  /* Infinite loop */
  for (;;)
  {
    // manually defines a critical region to ensure half-packets are never transmitted
//    taskENTER_CRITICAL();

	  HAL_GPIO_WritePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin, GPIO_PIN_SET);

    // create an empty buffer for the telemetry packet string
    char telemetry_string[200];

    // Generic temporary variable for use in sprintf() calls, etc.
    uint16_t str_len = 0;
    // Request semaphore access
    if (osSemaphoreWait(globalDataHandle, 100) != osOK) {
    	continue; // Until we can acquire a lock on the data, we do not want to read from it
    }

    // fill the buffer with the first half of the packet
    str_len = sprintf(telemetry_string, "%d,%s,%ld,%c,%s,%3.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%s,%.1f,%.4f,%.4f,%d,%s",
                      global_mission_data.TEAM_ID,      // team id (3174)
                      global_mission_data.MISSION_TIME, // mission time
                      global_mission_data.PACKET_COUNT, // packet count
                      global_mission_data.MODE,         // mode
                      global_mission_data.STATE,        // state
                      global_mission_data.ALTITUDE,     // calibrated altitude (m)
                      global_mission_data.TEMPERATURE,  // temperature (C)
                      global_mission_data.PRESSURE,     // pressure (kPa)
                      global_mission_data.VOLTAGE,      // battery voltage (V)
                      global_mission_data.GYRO_R,       // gyro roll (degrees/s)
                      global_mission_data.GYRO_P,       // gyro pitch (degrees/s)
                      global_mission_data.GYRO_Y,        // gyro yaw (degrees/s)
                      global_mission_data.ACCEL_R,                 // accelerometer roll (degrees/s^2)
                      global_mission_data.ACCEL_P,                 // accelerometer pitch (degrees/s^2)
                      global_mission_data.ACCEL_Y,                 // accelerometer yaw (degrees/s^2)
                      global_mission_data.GPS_TIME,                // GPS time
                      global_mission_data.GPS_ALTITUDE,            // GPS (absolute) altitude (m)
                      global_mission_data.GPS_LATITUDE,            // GPS latitude
                      global_mission_data.GPS_LONGITUDE,           // GPS longitude
                      global_mission_data.GPS_SATS,                // # of connected GPS satellites
                      global_mission_data.CMD_ECHO                 // tracks previously received command
    );
    // send the second half of the packet over UART
    HAL_UART_Transmit(&huart3, telemetry_string, str_len, HAL_MAX_DELAY);

    // increment packet count once the entire packet has been transmitted
    global_mission_data.PACKET_COUNT = global_mission_data.PACKET_COUNT + 1;

    write_SD(telemetry_string, str_len, "CanSat_Data_2026.csv");

    osSemaphoreRelease(globalDataHandle);
//    xSemaphoreGive(globalDataHandle);
    // exit the critical region once both packets have been sent
//    taskEXIT_CRITICAL();
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    HAL_GPIO_TogglePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin);

    osDelay(1000);
  }
  /* USER CODE END StartSendTelemetry */
}

/* USER CODE BEGIN Header_StartGNC */
/**
 * @brief Function implementing the guide_nav_ctrl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGNC */
void StartGNC(void const * argument)
{
  /* USER CODE BEGIN StartGNC */

  /* Infinite loop */
  for (;;)
  {

//	HAL_GPIO_TogglePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin);
    osDelay(250);
	  if (GPS_READY) {

		  GPS_READY = 0;
	  }
    osDelay(1);
  }
  /* USER CODE END StartGNC */
}

/* USER CODE BEGIN Header_StartInit */
/**
* @brief Function implementing the Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInit */
void StartInit(void const * argument)
{
  /* USER CODE BEGIN StartInit */
	init_SD();

	if (global_micro_sd_data.successfullyMounted) {
		uint32_t result = load_config_from_sd();
		if (result != APP_OK) {
			HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_SET);
			// Set default config
			set_default_config();
			save_config_to_sd();
		}
	}
	else {
		HAL_GPIO_WritePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin, GPIO_PIN_SET);
		set_default_config();
	}



	// Read Config file and update accordingly
	global_mission_data.ALTITUDE_OFFSET = global_config.ALTITUDE_OFFSET;
	global_mission_data.PACKET_COUNT	= global_config.PACKET_COUNT;

	strcpy(global_mission_data.MISSION_TIME, global_config.MISSION_TIME);
	strcpy(global_mission_data.STATE, global_config.STATE);

	// This lets you see the minimum amount of the stack was remaining at any time
	//  during a thread's execution.
	UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

	HAL_GPIO_WritePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin, GPIO_PIN_SET);



	// Init task has nothing left to do — delete itself
//	osThreadTerminate(osThreadGetId());
	vTaskDelete(NULL);

	for(;;);

  /* USER CODE END StartInit */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
