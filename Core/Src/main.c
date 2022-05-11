/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <soft_timer.h>
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ZekDe_funcs.h"
#include "mpu9250.h"
#include "usbd_cdc_if.h"
#include <stddef.h>
#include "math.h"
#include "Fusion.h"

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

/* USER CODE BEGIN PV */
rising_edge_detection_t s_red;

int16_t ai_accRaw[3] =
{ 0 };
int16_t ai_gyroRaw[3] =
{ 0 };

softTimer_t s_timerSend2PC;
softTimer_t s_timerFusionStep;

float f_gyroRes, f_accRes;

float af_accG[3], af_gyroDps[3], af_gyroRad[3], af_accUnit[3],
		af_accG_to_m_s2[3];

float af_gyroBias[3], af_accBias[3];
float f_norm_acc;

FusionBias s_fusionBias;
FusionAhrs s_fusionAhrs;

float f_samplePeriod = 0.01f;
_Bool o_fusionBiasActive = 0;

FusionVector3 s_gyroSensitivity =
{ .axis.x = 1.0f, .axis.y = 1.0f, .axis.z = 1.0f, };
FusionVector3 s_accSensitivity =
{ .axis.x = 1.0f, .axis.y = 1.0f, .axis.z = 1.0f, };

FusionVector3 s_calibratedGyro;
FusionVector3 s_calibratedAcc;
FusionEulerAngles s_eulerAngles;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void send2Pc(void);
void fusionTask(void);
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
	uint32_t tick = 0;
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
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */
	// to use scanf properly, without std C buffer
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	timerSetInterval(&s_timerSend2PC, 1000);
	timerStart(&s_timerSend2PC);
	timerSetInterval(&s_timerFusionStep, 10);
	timerStart(&s_timerFusionStep);

	MPU9250_reset(&hi2c1);
	HAL_Delay(100);
	MPU9250_init(&hi2c1, AFS_2G, GFS_250DPS);
	HAL_Delay(100);
	f_gyroRes = getGyroRes(AFS_2G);
	f_accRes = getAccRes(GFS_250DPS);
	MPU9250_calibrate(&hi2c1, af_gyroBias, af_accBias);
	HAL_Delay(100);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// Initialise gyroscope bias correction algorithm
	// stationary threshold = 0.5 degrees per second
	FusionBiasInitialise(&s_fusionBias, 2.0F, f_samplePeriod);
	// Initialise AHRS algorithm
	FusionAhrsInitialise(&s_fusionAhrs, 0.5f); // gain = 0.5

	while (1)
	{
		if (MPU9250_isDataReady(&hi2c1) == true)
		{
			getAccRaw(&hi2c1, ai_accRaw);
			getGyroRaw(&hi2c1, ai_gyroRaw);
			for (uint8_t i = 0; i < 3; ++i)
			{
				af_accG[i] = ai_accRaw[i] * f_accRes;
//				af_accG_to_m_s2[i] = af_accG[i] * 9.80665F;
				af_gyroDps[i] = ai_gyroRaw[i] * f_gyroRes;
//				af_gyroRad[i] = af_gyroDps[i] * 0.0174533F; // pi/180

			}
		}

		tick = HAL_GetTick();
//		timerCheck(&s_timerSend2PC, &tick, send2Pc);
		timerCheck(&s_timerFusionStep, &tick, fusionTask);
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	o_txCplt = 1;
	CDC_Transmit_FS(&ch, 1);
	while (o_txCplt)
		;
	return ch;
}

static void send2Pc(void)
{
	for (uint8_t i = 0; i < 3; ++i)
	{
		printf("acc(g) = %.3f\n", af_accG[i]);
	}
	for (uint8_t i = 0; i < 3; ++i)
	{
		printf("gyro(dps) = %.3f\n", af_gyroDps[i]);
	}
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

void fusionTask(void)
{
	static FusionVector3 s_uncalibratedGyro =
	{ .axis.x = 0.0F, .axis.y = 0.0f, .axis.z = 0.0f, };
	static FusionVector3 s_uncalibratedAcc =
	{ .axis.x = 0.0F, .axis.y = 0.0F, .axis.z = 1.0F, };

	s_uncalibratedGyro.axis.x = af_gyroDps[0];
	s_uncalibratedGyro.axis.y = af_gyroDps[1];
	s_uncalibratedGyro.axis.z = af_gyroDps[2];
	s_uncalibratedAcc.axis.x = af_accG[0];
	s_uncalibratedAcc.axis.y = af_accG[1];
	s_uncalibratedAcc.axis.z = af_accG[2];

	s_calibratedGyro = FusionCalibrationInertial(s_uncalibratedGyro,
			FUSION_ROTATION_MATRIX_IDENTITY, s_gyroSensitivity,
			FUSION_VECTOR3_ZERO);

	s_calibratedAcc = FusionCalibrationInertial(s_uncalibratedAcc,
			FUSION_ROTATION_MATRIX_IDENTITY, s_accSensitivity,
			FUSION_VECTOR3_ZERO);

	s_calibratedGyro = FusionBiasUpdate(&s_fusionBias, s_calibratedGyro);
	FusionAhrsUpdateWithoutMagnetometer(&s_fusionAhrs, s_calibratedGyro,
			s_calibratedAcc, f_samplePeriod);

	s_eulerAngles = FusionQuaternionToEulerAngles(
			FusionAhrsGetQuaternion(&s_fusionAhrs));
	o_fusionBiasActive = FusionBiasIsActive(&s_fusionBias);

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
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
