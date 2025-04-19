/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssp_common.h"
#include "ssp_comm.h"
#include "ssp_command.h"
#include "ssp_init.h"
#include "eps_faults.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_SAVE_INTERVAL 60000 // Save every 60 seconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
SSP_Frame TxFrame;
SSP_Frame RxFrame;
uint16_t AdcValues[16]; // Buffer to store ADC results for 16 channels
static uint8_t use_alternative_rs4851 = 0; // Toggle between USART1 and USART2 for RS485_1
static uint32_t last_save_time = 0; // For periodic EEPROM saving

// Telemetry and parameters for EEPROM storage
static EEPROM_Telemetry eepromTelemetry;
static EEPROM_Parameter eepromParameters[21]; // Assuming 21 parameters as in previous code
static uint8_t parameterCount = 21; // Number of parameters
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Set initial GPIO levels as specified
  HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_RESET); // PA12: Low
  HAL_GPIO_WritePin(RS12V_EN_GPIO_Port, RS12V_EN_Pin, GPIO_PIN_RESET); // PB2: Low
  HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET); // PB6: Low
  HAL_GPIO_WritePin(ADCS12V_EN_GPIO_Port, ADCS12V_EN_Pin, GPIO_PIN_RESET); // PB8: Low
  HAL_GPIO_WritePin(RS3V3_EN_GPIO_Port, RS3V3_EN_Pin, GPIO_PIN_RESET); // PB15: Low
  HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, GPIO_PIN_RESET); // PC7: Low
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET); // PC8: Low
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET); // PC9: Low
  HAL_GPIO_WritePin(UHF_EN_GPIO_Port, UHF_EN_Pin, GPIO_PIN_RESET); // PD7: Low
  HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); // PD12: Low
  HAL_GPIO_WritePin(PL_EN_GPIO_Port, PL_EN_Pin, GPIO_PIN_RESET); // PD13: Low
  HAL_GPIO_WritePin(RS5V_EN_GPIO_Port, RS5V_EN_Pin, GPIO_PIN_RESET); // PE2: Low
  HAL_GPIO_WritePin(PL5V_EN_GPIO_Port, PL5V_EN_Pin, GPIO_PIN_RESET); // PE3: Low
  HAL_GPIO_WritePin(ADCS5V_EN_GPIO_Port, ADCS5V_EN_Pin, GPIO_PIN_RESET); // PE4: Low
  HAL_GPIO_WritePin(XB12V_EN_GPIO_Port, XB12V_EN_Pin, GPIO_PIN_RESET); // PE8: Low
  HAL_GPIO_WritePin(CCU5V_EN_GPIO_Port, CCU5V_EN_Pin, GPIO_PIN_RESET); // PE10: Low
  HAL_GPIO_WritePin(BOOT0_GPIO_Port, BOOT0_Pin, GPIO_PIN_RESET); // PH3: Low

  // Initialize SSP
  SSP_Init(&huart1, &huart2, &huart3, RS4851_DE_GPIO_Port, RS4851_DE_Pin, RS4852_DE_GPIO_Port, RS4852_DE_Pin);

  // Initialize EEPROM and load stored data
  if (EEPROM_Init(&hi2c2) != HAL_OK) {
      char log_msg[] = "Failed_to_initialize_EEPROM\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
  }
  if (EEPROM_ReadTelemetry(&hi2c2, &eepromTelemetry) != HAL_OK) {
      char log_msg[] = "Failed_to_load_telemetry_from_EEPROM\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
  }
  if (EEPROM_ReadParameters(&hi2c2, eepromParameters, parameterCount) != HAL_OK) {
      char log_msg[] = "Failed_to_load_parameters_from_EEPROM\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
  }

  // Start ADC with DMA
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcValues, 16) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Update telemetry and parameters with the latest ADC readings
    SSP_UpdateTelemetryAndParameters(AdcValues);

    // Copy updated telemetry and parameters from SSP to EEPROM structures
    SSP_Telemetry *sspTelemetry = SSP_GetTelemetry();
    eepromTelemetry.Bus12V = sspTelemetry->Bus12V;
    eepromTelemetry.Bus5V = sspTelemetry->Bus5V;
    eepromTelemetry.Bus3V3 = sspTelemetry->Bus3V3;

    uint8_t sspParamCount;
    SSP_Parameter *sspParams = SSP_GetParameters(&sspParamCount);
    for (uint8_t i = 0; i < sspParamCount && i < parameterCount; i++) {
        eepromParameters[i].ParamId = sspParams[i].ParamId;
        eepromParameters[i].Value = sspParams[i].Value;
    }

    // Periodically save telemetry and parameters to EEPROM
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_save_time >= EEPROM_SAVE_INTERVAL) {
        if (EEPROM_WriteTelemetry(&hi2c2, &eepromTelemetry) == HAL_OK &&
            EEPROM_WriteParameters(&hi2c2, eepromParameters, parameterCount) == HAL_OK) {
            char log_msg[] = "Telemetry_and_parameters_saved_to_EEPROM\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
        } else {
            char log_msg[] = "Failed_to_save_telemetry/parameters_to_EEPROM\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
        }
        last_save_time = current_time;
    }

    // Send a PING command to all subsystems on RS485_1 (toggle between USART1 and USART2)
    UART_HandleTypeDef *rs4851_uart = use_alternative_rs4851 ? &huart2 : &huart1;
    const char *rs4851_label = use_alternative_rs4851 ? "RS485_1_(Alt,_USART2)" : "RS485_1_(USART1)";
    SSP_PackFrame(&TxFrame, SSP_ADDR_OBC, SSP_ADDR_ALL, SSP_CMD_PING, NULL, 0);
    if (SSP_SendFrame(rs4851_uart, RS4851_DE_GPIO_Port, RS4851_DE_Pin, &TxFrame) == HAL_OK) {
        char log_msg[80];
        snprintf(log_msg, sizeof(log_msg), "%s:_PING_Sent\n", rs4851_label);
        HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);

        HAL_StatusTypeDef status = SSP_ReceiveFrame(rs4851_uart, RS4851_DE_GPIO_Port, RS4851_DE_Pin, &RxFrame, 70); // 70 ms timeout per ICD
        if (status == HAL_OK) {
            snprintf(log_msg, sizeof(log_msg), "%s:_PING_Response_Received\n", rs4851_label);
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
            SSP_HandleCommand(&RxFrame, &TxFrame, rs4851_uart, &huart2, RS4851_DE_GPIO_Port, RS4851_DE_Pin);
        } else if (status == HAL_TIMEOUT) {
            snprintf(log_msg, sizeof(log_msg), "%s:_Timeout_-_Possible_open,_shorted,_or_idle_bus\n", rs4851_label);
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
        } else {
            snprintf(log_msg, sizeof(log_msg), "%s:_Error_receiving_frame\n", rs4851_label);
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
        }
    } else {
        char log_msg[80];
        snprintf(log_msg, sizeof(log_msg), "%s:_Failed_to_send_PING\n", rs4851_label);
        HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
    }

    // Toggle between USART1 and USART2 for RS485_1
    use_alternative_rs4851 = !use_alternative_rs4851;

    // Send a PING command on RS485_2 (USART3)
    SSP_PackFrame(&TxFrame, SSP_ADDR_OBC, SSP_ADDR_ALL, SSP_CMD_PING, NULL, 0);
    if (SSP_SendFrame(&huart3, RS4852_DE_GPIO_Port, RS4852_DE_Pin, &TxFrame) == HAL_OK) {
        char log_msg[80];
        snprintf(log_msg, sizeof(log_msg), "RS485_2_(USART3):_PING_Sent\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);

        HAL_StatusTypeDef status = SSP_ReceiveFrame(&huart3, RS4852_DE_GPIO_Port, RS4852_DE_Pin, &RxFrame, 70);
        if (status == HAL_OK) {
            snprintf(log_msg, sizeof(log_msg), "RS485_2_(USART3):_PING_Response_Received\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
            SSP_HandleCommand(&RxFrame, &TxFrame, &huart3, &huart2, RS4852_DE_GPIO_Port, RS4852_DE_Pin);
        } else if (status == HAL_TIMEOUT) {
            snprintf(log_msg, sizeof(log_msg), "RS485_2_(USART3):_Timeout_-_Possible_open,_shorted,_or_idle_bus\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
        } else {
            snprintf(log_msg, sizeof(log_msg), "RS485_2_(USART3):_Error_receiving_frame\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
        }
    } else {
        char log_msg[80];
        snprintf(log_msg, sizeof(log_msg), "RS485_2_(USART3):_Failed_to_send_PING\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)log_msg, strlen(log_msg), 100);
    }

    // Handle faults (interrupts) and poll remaining fault pins
    HandleFaults(&huart2, &huart2, RS4851_DE_GPIO_Port, RS4851_DE_Pin);
    PollFaults(&hi2c2, &huart2);

    HAL_Delay(10); // Adjust polling interval to avoid overwhelming the system
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; // Enable scanning for multiple channels
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV; // End of conversion for sequence
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE; // Continuous conversion for periodic sampling
  hadc1.Init.NbrOfConversion = 16; // 16 channels to match AdcValues array
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE; // Continuous DMA requests
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure ADC channels (16 channels as per AdcValues)
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  // Channel 1: PC0 (RS5V_I)
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 2: PC1 (PL5V_I)
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 3: PC2 (ADCS5V_I)
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 4: PC3 (GPS_I)
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 5: PA0 (M/OUT, multiplexed: SA1_I, SA3_V, SA2_V, SA1_V)
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 6: PA1 (SA2_I)
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 7: PA2 (SA3_I)
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 8: PA3 (XB12V_I)
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 9: PA4 (CCU5V_I)
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 10: PA5 (CCU_I)
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 11: PA6 (RS3V3_I)
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 12: PA7 (ADCS_I)
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 13: PC4 (PL_I)
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 14: PC5 (ADCS12V_I)
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 15: PB0 (UHF_I)
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 16: PB1 (OBC_I)
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC; // 100 kHz, adjusted for 80 MHz SYSCLK
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT; // Adjusted to 8-bit for standard SPI communication
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
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
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK) // Enable DE functionality for RS485_1
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RS5V_EN_Pin|PL5V_EN_Pin|ADCS5V_EN_Pin|XB12V_EN_Pin
                          |CCU5V_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS12V_EN_Pin|RS3V3_EN_Pin|GPS_EN_Pin|ADCS12V_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RS4852_DE_Pin|PL_EN_Pin|UHF_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S0_Pin|S1_Pin|S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADCS_EN_GPIO_Port, ADCS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOOT0_GPIO_Port, BOOT0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS5V_EN_Pin PL5V_EN_Pin ADCS5V_EN_Pin XB12V_EN_Pin
                           CCU5V_EN_Pin */
  GPIO_InitStruct.Pin = RS5V_EN_Pin|PL5V_EN_Pin|ADCS5V_EN_Pin|XB12V_EN_Pin
                          |CCU5V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RS12V_EN_Pin GPS_EN_Pin ADCS12V_EN_Pin */
  GPIO_InitStruct.Pin = RS12V_EN_Pin|GPS_EN_Pin|ADCS12V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RS12V_FLT_Pin XB12V_FLT_Pin PL5V_FLT_Pin */
  GPIO_InitStruct.Pin = RS12V_FLT_Pin|XB12V_FLT_Pin|PL5V_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CCU5V_FLT_Pin RS5V_FLT_Pin */
  GPIO_InitStruct.Pin = CCU5V_FLT_Pin|RS5V_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RS3V3_EN_Pin */
  GPIO_InitStruct.Pin = RS3V3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS3V3_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS3V3_FLT_Pin PL_FLT_Pin ADCS_FAULT_Pin OBC_FAULT_Pin */
  GPIO_InitStruct.Pin = RS3V3_FLT_Pin|PL_FLT_Pin|ADCS_FAULT_Pin|OBC_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CCU_FAULT_Pin */
  GPIO_InitStruct.Pin = CCU_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CCU_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS4852_DE_Pin PL_EN_Pin UHF_EN_Pin */
  GPIO_InitStruct.Pin = RS4852_DE_Pin|PL_EN_Pin|UHF_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ADCS_EN_Pin */
  GPIO_InitStruct.Pin = ADCS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ADCS_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_SS_Pin */
  GPIO_InitStruct.Pin = SPI3_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(SPI3_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SP13_SCK_Pin SPI3_MISO_Pin SP13_MOSI_Pin */
  GPIO_InitStruct.Pin = SP13_SCK_Pin|SPI3_MISO_Pin|SP13_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SYNC_PULSE_Pin */
  GPIO_InitStruct.Pin = SYNC_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SYNC_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UHF_FLT_Pin GPS_FLT_Pin ADCS12V_FLT_Pin ADCS5V_FLT_Pin */
  GPIO_InitStruct.Pin = UHF_FLT_Pin|GPS_FLT_Pin|ADCS12V_FLT_Pin|ADCS5V_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT0_Pin */
  GPIO_InitStruct.Pin = BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOT0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  while (1) {}
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
