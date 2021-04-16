/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
//MACRO DEFINITIONS
#define MPU6050_ADDR (0xD0)
#define WHO_AM_I (0x75)
#define MPU_PWR_MGMT (0x6B)
#define GYRO_CONFIG (0x1B)
#define ACCEL_CONFIG (0x1C)
#define ACCEL_XOUT_H (0x3B)
#define GYRO_XOUT_H (0x43)
#define SMPLRT_DIV_REG (0x19)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void MPU6050Init();
void AccRead();
void GyroRead();
void EnterLPowMode();
void ExitLPowMode();

/* Private user code ---------------------------------------------------------*/

//global variables
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;



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

  /* MPU6050 init */
  MPU6050Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//1. Exit low power mode
	//2. read accelerometer data.
	//3. read gyro data.
	//4. enter low power mode.
	//5. wait for 10ms
	//6. go to step 1
	  ExitLPowMode();
	  AccRead();
	  GyroRead();
	  EnterLPowMode();
	  HAL_Delay(10);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief Initilizes the sensor MPU6050
 *
 */
void MPU6050Init()
{
	uint8_t status;
	uint8_t dataBuffer;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I,1, &status, 1, 1000);

	if (status == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		dataBuffer = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_PWR_MGMT, 1,&dataBuffer, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		dataBuffer = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &dataBuffer, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		dataBuffer = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &dataBuffer, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		dataBuffer = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &dataBuffer, 1, 1000);
	}

}

/**
 * @brief Initilizes the sensor MPU6050
 * @retval None
 */
void AccRead()
{
	uint8_t Rec_Data[6];
	//the global variables will add upto 100 values for parameters
	for(int index=0;index<100;index++){
		// Read 6 BYTES of data starting from ACCEL_XOUT_H register

		HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

		Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

		// convert the RAW values into acceleration in 'g'
		//	we have to divide according to the Full scale value set in FS_SEL
		//	I have configured FS_SEL = 0. So I am dividing by 16384.0
		//	for more details check ACCEL_CONFIG Register

		Ax = Ax + (Accel_X_RAW/16384.0);
		Ay = Ay + (Accel_Y_RAW/16384.0);
		Az = Az + (Accel_Z_RAW/16384.0);
	}
	Ax /= 100;
	Ay /= 100;
	Az /= 100;
}

/**
 * @brief read gyro data
 * @retval None
 */

void GyroRead()
{
	uint8_t Rec_Data[6];
	//the global variables will add upto 100 values for parameters
	for(int index=0;index<100;index++){

		// Read 6 BYTES of data starting from GYRO_XOUT_H register

		HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 1000);

		Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

		/*** convert the RAW values into dps (°/s)
			 we have to divide according to the Full scale value set in FS_SEL
			 I have configured FS_SEL = 0. So I am dividing by 131.0
			 for more details check GYRO_CONFIG Register              ****/

		Gx = Gx + (Gyro_X_RAW/131.0);
		Gy = Gy + (Gyro_Y_RAW/131.0);
		Gz = Gz + (Gyro_Z_RAW/131.0);
	}
	Gx /= 100;
	Gy /= 100;
	Gz /= 100;

}

/**
 * @brief Enter low power run mode
 * @retval None
 */

void EnterLPowMode()
{
    /* 1. Each digital IP clock must be enabled or disabled by using the
                   RCC_APBxENR and RCC_AHBENR registers */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    /* 2. The frequency of the system clock must be decreased to not exceed the
          frequency of f_MSI range1. */
    SystemClock_Config();
    // Reinitialize peripherals dependent on clock speed
    MX_I2C1_Init();
    /* 3. The regulator is forced in low-power mode by software
          (LPRUN and LPSDSR bits set ) */
    PWR->CR &= ~PWR_CR_LPRUN; // Be sure LPRUN is cleared!

    PWR->CR |= PWR_CR_LPSDSR; // must be set before LPRUN
    PWR->CR |= PWR_CR_LPRUN; // enter low power run mode

}
/**
 * @brief exit low power mode
 * @retval None
 */

void ExitLPowMode()
{
    /* Enable Clocks */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Force the regulator into main mode */
    // Reset LPRUN bit
    PWR->CR &= ~( PWR_CR_LPRUN );
    // LPSDSR can be reset only when LPRUN bit = 0;
    PWR->CR &= ~( PWR_CR_LPSDSR );
    //system clock config
    SystemClock_Config();
    // Reinitialize peripherals dependent on clock speed
    MX_I2C1_Init();
    MPU6050Init();


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
