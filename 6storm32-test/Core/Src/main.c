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
#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  IMU_TEST_STARTING = 0,
  IMU_TEST_OK,
  IMU_TEST_I2C_ERROR,
  IMU_TEST_WRONG_ID,
  IMU_TEST_CONFIG_ERROR
} IMU_TestState;

/* Watch imu_test in CubeIDE Live Expressions, even without USB connected.
 * Raw values are signed sensor counts. Scaled units are included in the names.
 * A sample is valid only while state == IMU_TEST_OK; errors retain the last data.
 */
typedef struct
{
  IMU_TestState state;
  uint8_t who_am_i;
  uint8_t i2c_bus;
  uint8_t i2c_address;
  HAL_StatusTypeDef last_hal_status;
  uint32_t last_i2c_error;
  uint32_t samples;
  uint32_t errors;
  uint32_t last_sample_ms;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  int16_t temperature_raw;
  int32_t accel_mg[3];
  int32_t gyro_mdps[3];
  int32_t temperature_centi_c;
} IMU_TestStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* STorM32 v1.30 onboard LEDs: high = on, low = off.
 * MX_GPIO_Init() configures PB12/PB13 as push-pull outputs, initially low.
 */
#define LED_GREEN_PIN GPIO_PIN_12
#define LED_RED_PIN GPIO_PIN_13
#define LED_PORT GPIOB
#define IMU_LED_INTERVAL_MS 250U

/* Measured on the connected board with the external IMU unplugged:
 * onboard MPU6050 = I2C1 PB6=SCL, PB7=SDA, address 0x69.
 * This differs from the reference v1.30 schematic (I2C2, address 0x68).
 * STM32 HAL expects the 7-bit device address shifted left by one.
 * WHO_AM_I remains 0x68 even when the device address is 0x69.
 * Register definitions: TDK RM-MPU-6000A-00, revision 4.2.
 */
#define IMU_I2C_HANDLE hi2c1
#define MPU6050_ADDRESS_7BIT 0x69U
#define MPU6050_ADDRESS (MPU6050_ADDRESS_7BIT << 1)
#define MPU6050_WHO_AM_I 0x75U
#define MPU6050_EXPECTED_ID 0x68U
#define MPU6050_PWR_MGMT_1 0x6BU
#define MPU6050_PWR_MGMT_2 0x6CU
#define MPU6050_SMPLRT_DIV 0x19U
#define MPU6050_CONFIG 0x1AU
#define MPU6050_GYRO_CONFIG 0x1BU
#define MPU6050_ACCEL_CONFIG 0x1CU
#define MPU6050_ACCEL_XOUT_H 0x3BU
#define IMU_I2C_TIMEOUT_MS 50U
#define IMU_SAMPLE_INTERVAL_MS 20U
#define IMU_REPORT_INTERVAL_MS 250U
#define IMU_RETRY_INTERVAL_MS 1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Volatile keeps the live diagnostic values visible to the debugger. */
volatile IMU_TestStatus imu_test;
extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t imu_initialized;
static uint32_t imu_last_sample_ms;
static uint32_t imu_last_report_ms;
static uint32_t imu_last_retry_ms;
static uint32_t imu_last_led_ms;
/* USB retains this buffer until transmission completes; never use a stack buffer. */
static char imu_usb_line[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t IMU_TestInit(void);
static uint8_t IMU_TestRead(void);
static void IMU_TestReport(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Record failed HAL transactions without treating old samples as fresh data. */
static uint8_t IMU_CheckTransfer(HAL_StatusTypeDef status)
{
  imu_test.last_hal_status = status;
  imu_test.last_i2c_error = HAL_I2C_GetError(&IMU_I2C_HANDLE);
  if (status != HAL_OK)
  {
    imu_test.state = IMU_TEST_I2C_ERROR;
    imu_test.errors++;
    return 0U;
  }
  return 1U;
}

static uint8_t IMU_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
  return IMU_CheckTransfer(HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, MPU6050_ADDRESS, reg,
      I2C_MEMADD_SIZE_8BIT, data, length, IMU_I2C_TIMEOUT_MS));
}

static uint8_t IMU_WriteRegister(uint8_t reg, uint8_t value)
{
  return IMU_CheckTransfer(HAL_I2C_Mem_Write(&IMU_I2C_HANDLE, MPU6050_ADDRESS, reg,
      I2C_MEMADD_SIZE_8BIT, &value, 1U, IMU_I2C_TIMEOUT_MS));
}

static uint8_t IMU_TestInit(void)
{
  uint8_t identity = 0U;
  /* Reset before configuring so this test does not inherit an earlier setup. */
  static const uint8_t settings[][2] =
  {
    {MPU6050_PWR_MGMT_1, 0x01U}, /* Wake, temperature enabled, X-gyro PLL clock. */
    {MPU6050_PWR_MGMT_2, 0x00U}, /* Enable all accelerometer and gyro axes. */
    {MPU6050_SMPLRT_DIV, 19U},   /* 1 kHz / (1 + 19) = 50 Hz with DLPF enabled. */
    {MPU6050_CONFIG, 0x03U},     /* DLPF: accel 44 Hz, gyro 42 Hz. */
    {MPU6050_GYRO_CONFIG, 0x00U},/* +/-250 deg/s: 131 counts per deg/s. */
    {MPU6050_ACCEL_CONFIG, 0x00U}/* +/-2 g: 16384 counts per g. */
  };

  imu_test.i2c_bus = (IMU_I2C_HANDLE.Instance == I2C1) ? 1U : 2U;
  imu_test.i2c_address = MPU6050_ADDRESS_7BIT;
  imu_test.who_am_i = 0U;
  if (!IMU_ReadRegisters(MPU6050_WHO_AM_I, &identity, 1U))
  {
    return 0U;
  }
  imu_test.who_am_i = identity;
  if (identity != MPU6050_EXPECTED_ID)
  {
    imu_test.state = IMU_TEST_WRONG_ID;
    imu_test.errors++;
    return 0U;
  }
  if (!IMU_WriteRegister(MPU6050_PWR_MGMT_1, 0x80U))
  {
    return 0U;
  }
  HAL_Delay(100U); /* Device reset settling time. */

  for (uint32_t i = 0U; i < sizeof(settings) / sizeof(settings[0]); i++)
  {
    uint8_t actual = 0U;
    if (!IMU_WriteRegister(settings[i][0], settings[i][1]))
    {
      return 0U;
    }
    if (settings[i][0] == MPU6050_PWR_MGMT_1)
    {
      HAL_Delay(100U); /* Allow the gyro and PLL to settle after waking. */
    }
    if (!IMU_ReadRegisters(settings[i][0], &actual, 1U))
    {
      return 0U;
    }
    if (actual != settings[i][1])
    {
      imu_test.state = IMU_TEST_CONFIG_ERROR;
      imu_test.errors++;
      return 0U;
    }
  }

  /* Confirm a full sample can be read before indicating success. */
  return IMU_TestRead();
}

static int16_t IMU_DecodeSigned(const uint8_t *bytes)
{
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static uint8_t IMU_TestRead(void)
{
  uint8_t data[14];
  /* A single burst keeps accel, temperature, and gyro from the same sample. */
  if (!IMU_ReadRegisters(MPU6050_ACCEL_XOUT_H, data, sizeof(data)))
  {
    return 0U;
  }
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    imu_test.accel_raw[axis] = IMU_DecodeSigned(&data[axis * 2U]);
    imu_test.gyro_raw[axis] = IMU_DecodeSigned(&data[8U + axis * 2U]);
    /* Integer output avoids needing floating-point printf support. */
    imu_test.accel_mg[axis] = (int32_t)imu_test.accel_raw[axis] * 1000 / 16384;
    imu_test.gyro_mdps[axis] = (int32_t)imu_test.gyro_raw[axis] * 1000 / 131;
  }
  imu_test.temperature_raw = IMU_DecodeSigned(&data[6]);
  imu_test.temperature_centi_c = (int32_t)imu_test.temperature_raw * 100 / 340 + 3653;
  imu_test.last_sample_ms = HAL_GetTick();
  imu_test.samples++;
  imu_test.state = IMU_TEST_OK;
  return 1U;
}

static void IMU_TestReport(void)
{
  /* Skip reports if USB is absent or busy, keeping sampling independent of it.
   * Guard class-data access because USB reset/disconnect can free it in an IRQ.
   * This function is the only CDC sender; do not overwrite an in-flight buffer.
   */
  uint32_t irq_mask = __get_PRIMASK();
  __disable_irq();
  USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
  uint8_t ready = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
                  cdc != NULL && cdc->TxState == 0U);
  __set_PRIMASK(irq_mask);
  if (!ready)
  {
    return;
  }

  int length;
  if (imu_test.state == IMU_TEST_OK)
  {
    length = snprintf(imu_usb_line, sizeof(imu_usb_line),
        "IMU OK bus=I2C%u addr=0x%02X id=0x%02X n=%lu accel_mg=[%ld,%ld,%ld] "
        "gyro_mdps=[%ld,%ld,%ld] temp_centiC=%ld errors=%lu\r\n",
        (unsigned int)imu_test.i2c_bus, (unsigned int)imu_test.i2c_address,
        (unsigned int)imu_test.who_am_i, (unsigned long)imu_test.samples,
        (long)imu_test.accel_mg[0], (long)imu_test.accel_mg[1], (long)imu_test.accel_mg[2],
        (long)imu_test.gyro_mdps[0], (long)imu_test.gyro_mdps[1], (long)imu_test.gyro_mdps[2],
        (long)imu_test.temperature_centi_c, (unsigned long)imu_test.errors);
  }
  else
  {
    length = snprintf(imu_usb_line, sizeof(imu_usb_line),
        "IMU ERROR bus=I2C%u addr=0x%02X state=%u id=0x%02X hal=%u i2c=0x%08lX errors=%lu\r\n",
        (unsigned int)imu_test.i2c_bus, (unsigned int)imu_test.i2c_address,
        (unsigned int)imu_test.state, (unsigned int)imu_test.who_am_i,
        (unsigned int)imu_test.last_hal_status, (unsigned long)imu_test.last_i2c_error,
        (unsigned long)imu_test.errors);
  }
  if (length <= 0 || (size_t)length >= sizeof(imu_usb_line))
  {
    return;
  }

  /* Recheck after formatting, then submit while USB IRQs cannot invalidate cdc.
   * Interrupts are masked only for the checks/submission, not for formatting.
   */
  irq_mask = __get_PRIMASK();
  __disable_irq();
  cdc = hUsbDeviceFS.pClassData;
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
      cdc != NULL && cdc->TxState == 0U)
  {
    (void)CDC_Transmit_FS((uint8_t *)imu_usb_line, (uint16_t)length);
  }
  __set_PRIMASK(irq_mask);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* PB5 stays low during startup to force USB disconnect before enumeration. */
  HAL_Delay(100U);
  imu_initialized = IMU_TestInit();
  /* PB5 is open drain: high releases the USB D+ pull-up control. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  imu_last_sample_ms = HAL_GetTick();
  imu_last_retry_ms = imu_last_sample_ms;
  imu_last_report_ms = imu_last_sample_ms;
  imu_last_led_ms = imu_last_sample_ms;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if (!imu_initialized)
    {
      /* Retry initialization once per second; never halt on a sensor failure. */
      if ((uint32_t)(now - imu_last_retry_ms) >= IMU_RETRY_INTERVAL_MS)
      {
        imu_initialized = IMU_TestInit();
        now = HAL_GetTick();
        imu_last_retry_ms = now;
        imu_last_sample_ms = now;
      }
    }
    else if ((uint32_t)(now - imu_last_sample_ms) >= IMU_SAMPLE_INTERVAL_MS)
    {
      imu_initialized = IMU_TestRead();
      now = HAL_GetTick();
      imu_last_sample_ms = now;
      if (!imu_initialized)
      {
        imu_last_retry_ms = now;
      }
    }

    /* Red = failed identity/configuration/read. Green heartbeat = reads succeed.
     * LEDs indicate communication status; inspect motion values to test sensing.
     */
    HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN,
                     imu_initialized ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (!imu_initialized)
    {
      HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    }
    else if ((uint32_t)(now - imu_last_led_ms) >= IMU_LED_INTERVAL_MS)
    {
      HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_PIN);
      imu_last_led_ms = now;
    }

    if ((uint32_t)(now - imu_last_report_ms) >= IMU_REPORT_INTERVAL_MS)
    {
      imu_last_report_ms = now;
      IMU_TestReport();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3599;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
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
