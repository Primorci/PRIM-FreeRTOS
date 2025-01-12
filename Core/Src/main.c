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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "task.h"

#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int gyroStatus;  // 0 = OK, 1 = Sensor Read Error, 2 = Mutex Error
    int accelStatus; // 0 = OK, 1 = Sensor Read Error, 2 = Mutex Error
} SystemStatus;

typedef struct {
    int16_t gyroData[3];  // x, y, z axes
    int16_t accelData[3]; // x, y, z axes
} STM_SensorData;

typedef struct {
    uint8_t packetID;    // Identifier for the packet
    uint16_t dataSize;   // Size of the payload (gyroscope + accelerometer data)
    uint32_t timestamp;  // Optional: Timestamp for the data (in ms)
} PacketHeader;

typedef struct {
    PacketHeader header; // Packet header
    STM_SensorData data;     // Sensor data (gyroscope + accelerometer)
} SensorPacket;

typedef struct {
    int8_t dir_status;
    int8_t dir_RoadType;
    uint8_t status;
    uint8_t roadType;
} GPIOControlParams;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_MODE 0
#define DEBUG_MODE   1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint8_t currentMode = DISPLAY_MODE; // Start in display mode

STM_SensorData sharedSensorData;
SemaphoreHandle_t dataMutex;
SystemStatus sharedStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint8_t i2c1_pisiRegister(uint8_t, uint8_t, uint8_t);
HAL_StatusTypeDef i2c1_beriRegistre(uint8_t, uint8_t, uint8_t*, uint8_t);
void initOrientation(void);

uint8_t spi1_beriRegister(uint8_t);
HAL_StatusTypeDef spi1_beriRegistre(uint8_t, uint8_t*, uint8_t);
void spi1_pisiRegister(uint8_t, uint8_t);
void initGyro(void);

void initMutex(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t i2c1_pisiRegister(uint8_t naprava, uint8_t reg, uint8_t podatek) {
    naprava <<= 1;
    return HAL_I2C_Mem_Write(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, &podatek, 1, 10);
}

HAL_StatusTypeDef i2c1_beriRegistre(uint8_t naprava, uint8_t reg, uint8_t* podatek, uint8_t dolzina) {
    if ((dolzina>1)&&(naprava==0x19))  // ce je naprava 0x19 moramo postaviti ta bit, ce zelimo brati vec zlogov
        reg |= 0x80;
    naprava <<= 1;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, podatek, dolzina, dolzina);

	return status;
}

void initOrientation() { // ne pozabit klicati te funkcije
    // inicializiraj pospeskometer
    i2c1_pisiRegister(0x19, 0x20, 0x27);  // zbudi pospeskometer in omogoci osi
    i2c1_pisiRegister(0x19, 0x23, 0x88);  // nastavi posodobitev samo ko se prebere vrednost ter visoko locljivost
}

uint8_t spi1_beriRegister(uint8_t reg) {
    uint16_t buf_out, buf_in;
    reg |= 0x80; // najpomembnejsi bit na 1
    buf_out = reg; // little endian, se postavi na pravo mesto ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&buf_out, (uint8_t*)&buf_in, 2, 2); // blocking posiljanje ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    return buf_in >> 8; // little endian...
}

void spi1_pisiRegister(uint8_t reg, uint8_t vrednost) {
    uint16_t buf_out;
    buf_out = reg | (vrednost<<8); // little endian, se postavi na pravo mesto ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&buf_out, 2, 2); // blocking posiljanje ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

HAL_StatusTypeDef spi1_beriRegistre(uint8_t reg, uint8_t* buffer, uint8_t velikost) {
    reg |= 0xC0; // najpomembnejsa bita na 1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_StatusTypeDef status_SPI_Transmit = HAL_SPI_Transmit(&hspi1, &reg, 1, 10); // blocking posiljanje....
    HAL_StatusTypeDef status_SPI_Receive = HAL_SPI_Receive(&hspi1,  buffer, velikost, velikost); // blocking posiljanje....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    return status_SPI_Transmit != HAL_OK ? HAL_ERROR : status_SPI_Receive != HAL_OK ? HAL_ERROR : HAL_OK;
}

void initGyro() { // ne pozabit klicat te funkcije
    // preverimo ali smo "poklicali" pravi senzor
    uint8_t cip = spi1_beriRegister(0x0F);
    if (cip!=0xD4 && cip!=0xD3) {
        for (;;);
    }
    spi1_pisiRegister(0x20, 0x0F); // zbudi ziroskop in omogoci osi
}

void initMutex(void){
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Error_Handler(); // Handle error if mutex creation fails
    }
}

void getGyroData(void *pvParameters) {
	while (1) {
		int16_t gyro[3];
		if (spi1_beriRegistre(0x28, (uint8_t *)gyro, 6) == HAL_OK) {
			// Protect shared data with mutex
			if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
				memcpy(sharedSensorData.gyroData, gyro, sizeof(gyro));
				xSemaphoreGive(dataMutex);
				sharedStatus.gyroStatus = 0; // Update status to OK
			} else {
				sharedStatus.gyroStatus = 2; // Mutex acquisition error
			}
		} else {
			sharedStatus.gyroStatus = 1; // Sensor read error
			for(uint8_t i = 0; i < 3; i++)
				gyro[i] = 0;
			if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
				memcpy(sharedSensorData.gyroData, gyro, sizeof(gyro));
				xSemaphoreGive(dataMutex);
				sharedStatus.gyroStatus = 0; // Update status to OK
			} else {
				sharedStatus.gyroStatus = 2; // Mutex acquisition error
			}
		}
		vTaskDelay(pdMS_TO_TICKS(50)); // Adjust delay as needed
	}
}

void getAccelData(void *pvParameters) {
	while (1) {
		int16_t accel[3];
		if (i2c1_beriRegistre(0x19, 0x28, (uint8_t *)accel, 6) == HAL_OK) {
			// Protect shared data with mutex
			if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
				memcpy(sharedSensorData.accelData, accel, sizeof(accel));
				xSemaphoreGive(dataMutex);
				sharedStatus.accelStatus = 0; // Update status to OK
			} else {
				sharedStatus.accelStatus = 2; // Mutex acquisition error
			}
		} else {
			sharedStatus.accelStatus = 1; // Sensor read error
			for(uint8_t i = 0; i < 3; i++)
				accel[i] = 0;
			if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
				memcpy(sharedSensorData.accelData, accel, sizeof(accel));
				xSemaphoreGive(dataMutex);
				sharedStatus.accelStatus = 0; // Update status to OK
			} else {
				sharedStatus.accelStatus = 2; // Mutex acquisition error
			}
		}
		vTaskDelay(pdMS_TO_TICKS(50)); // Adjust delay as needed
	}
}

void sendData(void *pvParameters) {
	SensorPacket packet;

	while (1) {
		// Safely copy shared sensor data
		xSemaphoreTake(dataMutex, portMAX_DELAY);
		packet.data = sharedSensorData;
		xSemaphoreGive(dataMutex);

		// Populate the header
		packet.header.packetID = 0xab;
		packet.header.dataSize = sizeof(SensorData);
		packet.header.timestamp = xTaskGetTickCount();

		// Send the complete packet (header + data)
		while (CDC_Transmit_FS((uint8_t *)&packet, sizeof(SensorPacket)));

		vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay as needed
	}
}

void update_pwm_brightness(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t brightness_level) {
    uint32_t ccr_value = (brightness_level * (999 + 1)) / 100;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr_value);
}

void checkDir(uint8_t value, int8_t *dir, int8_t max, int8_t min){
	if(value >= max)
		*dir = -1;
	else if(value <= min)
		*dir = 1;
}

void GPIO_control(void *pvParameters){
	GPIOControlParams *params = (GPIOControlParams *)pvParameters;

	int8_t dir_status = params->dir_status;
	int8_t dir_RoadType = params->dir_RoadType;
	uint8_t status = params->status;
	uint8_t roadType = params->roadType;
    uint8_t toggle = 0;

	while(1){
		if (currentMode == DISPLAY_MODE) {
			if(isDefined){
				// Update LED for danger status
				update_pwm_brightness(&htim4, TIM_CHANNEL_3, recivedData.danger ? 100 : 0);

				// Update LED for status
				update_pwm_brightness(&htim4, TIM_CHANNEL_2, (uint8_t)status);

				// Update LEDs for road type based on the first character
				if (recivedData.roadType[0] == 'A') {
					update_pwm_brightness(&htim4, TIM_CHANNEL_1, 0);
					update_pwm_brightness(&htim4, TIM_CHANNEL_4, (uint8_t)roadType);
				} else {
					update_pwm_brightness(&htim4, TIM_CHANNEL_4, 0);
					update_pwm_brightness(&htim4, TIM_CHANNEL_1, (uint8_t)roadType);
				}

				// Update status and roadType values dynamically
				status += recivedData.danger
							  ? 10 * (recivedData.dangerProximity / 20) * dir_status
							  : 10 * dir_status;
				roadType += 10 * ((100 - recivedData.roadQuality) / 20) * dir_RoadType;

				// Ensure values remain within bounds
				checkDir(status, &dir_status, 100, 0);
				checkDir(roadType, &dir_RoadType, 100, 0);
			}
			else {
				// Fallback behavior: Toggle TIM_CHANNEL_3
				toggle = !toggle;
				update_pwm_brightness(&htim4, TIM_CHANNEL_3, toggle ? 100 : 0);

				update_pwm_brightness(&htim4, TIM_CHANNEL_4, 0);
				update_pwm_brightness(&htim4, TIM_CHANNEL_1, 0);
				update_pwm_brightness(&htim4, TIM_CHANNEL_2, 0);
			}
		}
		else{
			toggle = !toggle;
			update_pwm_brightness(&htim4, TIM_CHANNEL_2, toggle ? 100 : 0);
			update_pwm_brightness(&htim4, TIM_CHANNEL_4, 0);
			update_pwm_brightness(&htim4, TIM_CHANNEL_1, 0);
			update_pwm_brightness(&htim4, TIM_CHANNEL_3, 0);
		}
		// Delay for consistent updates
		vTaskDelay(pdMS_TO_TICKS(100));
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  GPIOControlParams gpioParams = {
      .dir_status = 1,
      .dir_RoadType = 1,
      .status = 0,
      .roadType = 0
  };

  initMutex();

  __HAL_I2C_ENABLE(&hi2c1);
  initOrientation();

  __HAL_SPI_ENABLE(&hspi1);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // CS postavimo na 1
  initGyro();

  // zazenemo casovnik
  HAL_TIM_Base_Start(&htim4);

  // zazenemo PWM - neinvertirani izhodi
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  xTaskCreate(
		getGyroData,       /* Function that implements the task. */
		"getGyroData",          /* Text name for the task. */
		128,      /* Stack size in words, not bytes. */
		NULL,    /* Parameter passed into the task. */
		1,/* Priority at which the task is created. */
		NULL);      /* Used to pass out the created task's handle. */
  xTaskCreate(getAccelData, "getAccelData", 128, NULL, 1, NULL);
  xTaskCreate(getAccelData, "getAccelData", 128, NULL, 1, NULL);
  xTaskCreate(sendData, "sendData", 128, NULL, 1, NULL);
  xTaskCreate(GPIO_control, "GPIO_control", 128, (void *)&gpioParams, 1, NULL);
  vTaskStartScheduler();
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
