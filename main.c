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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "bmp280.h"
#include "control.h"
#include "pid.h"
#include "nrf24l01p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
uint16_t pwm =1000;
////////////MPU6050 PV///////////////
int MPU6050_adress;
int16_t MPU6050_AccelRaw[3];
int16_t MPU6050_GyroRaw[3];
float MPU6050_AccelIng[3];
float MPU6050_GyroIns[3];
float ROLL;
float PITCH;
float a_norm;
MPU6050InitStatus MPU6050_InitValue;
volatile float GyroInsForControl;//bu controldeki anlik degeri veriyor.
//////////////////////////////////////
///////////BMP280 PV//////////////////
int BMP280_adress;
float temperature;
float pressure;
float p0;
float altitude;
float filtered_altitude;
///////////////////////////////////////
////////////NRF PVS//////////////////////
uint8_t RXBuffer[4];
int8_t rollFromJoystick;
int8_t pitchFromJoystick;
int8_t yawFromJoystick;
int16_t altitudeFromJoystick=1100;
uint8_t status;
uint32_t lastReceiveTime;
uint8_t nrf_data_received;
//////////CONTROL PV/////////////////////
volatile float pidROLLOutput;
motors_t motors;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motormixer(float throttle,float rollOutput){
	motors.motor1 = throttle - rollOutput; // ön sağ
	motors.motor2 = throttle + rollOutput; // ön sol
	motors.motor3 = throttle - rollOutput; // arka sağ
	motors.motor4 = throttle + rollOutput; // arka sol
}
void setMotorPWM(){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,(uint16_t)motors.motor1);//on sağ
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(uint16_t)motors.motor2);//on sol
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,(uint16_t)motors.motor3);//arka sag
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,(uint16_t)motors.motor4);//arka sol
}
void NRF_Receive(){
	status=nrf24l01p_get_status();

	 nrf_data_received = 1;
		   if (status & (1 << 6)) {
			   lastReceiveTime = HAL_GetTick();
			   nrf24l01p_rx_receive(RXBuffer);
			   rollFromJoystick =(int8_t) RXBuffer[0];
			   pitchFromJoystick =(int8_t) RXBuffer[1];
			   yawFromJoystick =(int8_t) RXBuffer[2];
			   altitudeFromJoystick= ((int16_t)RXBuffer[3]) * 10;
			   nrf24l01p_clear_rx_dr();
			   HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, 1);
		   }
		   if (nrf_data_received) {
		          if (HAL_GetTick() - lastReceiveTime > 1000) {
		              // 1 saniyedir veri yok

		              nrf_data_received = 0;

		              // Güvenli moda geç (çok önemli drone için)
		              rollFromJoystick  = 0;
		              pitchFromJoystick = 0;
		              yawFromJoystick   = 0;
		              altitudeFromJoystick = 0;
		          }
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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
/////TIMER CODES///////////////////////////////////////////////////////////////////////
HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);
HAL_GPIO_WritePin(led_GPIO_Port,led_Pin, 1);

HAL_Delay(2500);

// Sonra min
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);

HAL_Delay(2500);
HAL_GPIO_WritePin(led_GPIO_Port,led_Pin, 0);//mavi led sönünce kalibrasyon bitti
/////////////MPU6050 CODES////////////////////
MPU6050_adress = MPU6050_ScanDeviceID(&hi2c1);
MPU6050_InitValue = MPU6050_Init(&hi2c1, MPU6050_ACCEL_RANGE_8G, MPU6050_GYRO_RANGE_2000);
HAL_Delay(20);
CalibrateGyroBias(&hi2c1, MPU6050_GYRO_RANGE_2000, 1000);
InitRollAndPitchFromAccel(&hi2c1, MPU6050_ACCEL_RANGE_8G);
////////////////////////////////////////////////////////////////////////
////////////////BMP280 CODES////////////////////////////////////////////
/*BMP280_adress = BMP280_ScanDevıceID(&hi2c1);
BMP280_Init(&hi2c1);
HAL_Delay(25);//init sonrasi hazir olamiyor ve p0a cop deger veriyordu
ReadCalibrationData(&hi2c1);
BMP280_ReadSensorData(&hi2c1, &temperature, &p0);//baslangic basinc degeri bulundugu yukseklik = 0 alabilmesi icin.
BMP280_ReadSensorData(&hi2c1, &temperature, &pressure);
filtered_altitude = PressureToAltitude(pressure, p0);*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////CONTROL CODES///////////////////7
Control_pid_Roll_Rate_Init();
//////////////////////NRF CODES////////////////////
nrf24l01p_rx_init(2500, _250kbps);
HAL_Delay(50);
//pid rolloutput -10 oluyor -30 gönderince.

HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, 1);//yeşil led yanınca döngüye girdik.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_WritePin(orange_GPIO_Port, orange_Pin, 1);
	  ///////////////MPU6050 CODES////////////////
	  MPU6050_getAccelValue(&hi2c1, MPU6050_AccelRaw);
	  MPU6050_getGyroValue(&hi2c1, MPU6050_GyroRaw);
	  MPU6050_getAccelInG(MPU6050_AccelRaw, MPU6050_ACCEL_RANGE_8G, MPU6050_AccelIng);
	  MPU6050_getGyroIns(MPU6050_GyroRaw, MPU6050_GYRO_RANGE_2000, MPU6050_GyroIns);
	  UpdateRollAndPitch(&hi2c1, MPU6050_ACCEL_RANGE_8G, MPU6050_GYRO_RANGE_2000, &ROLL, &PITCH,&a_norm);
	  GyroInsForControl=gyroInsMinesBiasForRateRoll(&hi2c1, MPU6050_GYRO_RANGE_2000);//controlupdate icine verecegiz*/
	  ////////////////////////////////////////////////////////////////////////////////////////////////////
	  ////////////////BMP280codes/////////////////
      /*BMP280_ReadSensorData(&hi2c1, &temperature, &pressure);
      altitude = PressureToAltitude(pressure, p0);
      filtered_altitude = FilterAltitude(altitude, filtered_altitude);*/
      //////////////////////////////////////////////////////////////////////////////////////////////
               NRF_Receive();
              /* __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, altitudeFromJoystick);
               __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, altitudeFromJoystick);
               __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, altitudeFromJoystick);
               __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, altitudeFromJoystick);*/


              ////burada maximum ve minimum throtleları da teyit ettikten sonra tam dengede kalınan pwm değerini bulacağız(kumanda ile veri göndererek)
              ////o hover throtleını joystickin orta konumu olarak ayarlayacağız.pidde base throttle olacak aynı zamanda. //1400 ile tunning yapılacak.
              ////ardından dronu bağlayıp rate pid algoritmasını yazacağız.
              ////pid algoritması yaparken rate pid için kumandadan komut gitmeyecek elle iteceğim geri gelme titreşimine baglı olarak p i d ayarlanacak.
              ////önerilen değerler kp 0.08f kd 0.003f
              ////ardından angle pid roll ve pitche geçilecek angle pid nin outputu rate pidnin setpointi olarak gelecek
              ////VE UÇUŞ TESTİ.
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
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
  sConfigOC.Pulse = 1000;
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
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1999;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ce_Pin|nss_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, green_led_Pin|orange_Pin|led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ce_Pin nss_Pin */
  GPIO_InitStruct.Pin = ce_Pin|nss_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : green_led_Pin orange_Pin led_Pin */
  GPIO_InitStruct.Pin = green_led_Pin|orange_Pin|led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if (htim ->Instance == TIM6) {
    pidROLLOutput=Control_pid_Roll_Rate_Update(rollFromJoystick, GyroInsForControl,ROLL);
    motormixer(altitudeFromJoystick, pidROLLOutput);//throtle joystickden alacağız.
    setMotorPWM();
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
