/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "i2c.h"
#include "gpio.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "accel.h"
#include "MahonyAHRS.h"
#include "quatern2rotMat.h"
#include "tcAcc.h"
#include "linVel.h"
#include "linPos.h"
#include "KalmanFilter.h"
#include "matrix_ops.h"
#include <math.h>
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
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
static  float acc_x, acc_y, acc_z;
static float gyr_x, gyr_y, gyr_z;
static  float R[3][3];
float tcAcc[3];
float linAcc[3];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
        while (HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10) != HAL_OK) {};
 return ch;
}
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
    MahonyAHRS ahrs;
    Mahony_Init(&ahrs, 0.25f, 1.0f, 0.0f);
    uint8_t 	imu_readings[IMU_NUMBER_OF_BYTES];
    uint8_t 	gyro_readings[GYRO_NUMBER_OF_BYTES];
    int16_t 	accel_data[3];
    int16_t 	gyro_data[3];
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  BNO055_Init_I2C(&hi2c1);
  KalmanFilter kf_gyr_x, kf_gyr_y, kf_gyr_z;
  LinVel_Init();
  LinPos_Init();
  float samplePeriod = 0.25f;

  //KalmanFilter_Init(&kf_acc_x, 0.05f, 0.01f, 0);
  //KalmanFilter_Init(&kf_acc_y, 0.05f, 0.01f, 0);
  //KalmanFilter_Init(&kf_acc_z, 0.05f, 0.01f, 9.81);

  KalmanFilter_Init(&kf_gyr_x, 0.005f, 0.01f, 0.001);
  KalmanFilter_Init(&kf_gyr_y, 0.005f, 0.01f, 0.001);
  KalmanFilter_Init(&kf_gyr_z, 0.005f, 0.01f, 0.001);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(250);
	  GetAccelData(&hi2c1, (uint8_t*)imu_readings);
	  accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);      // Turn the MSB and LSB into a signed 16-bit value
	  accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
	  accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
	  acc_x = ((float)(accel_data[0]))/100.0f; //m/s2
	  acc_y = ((float)(accel_data[1]))/100.0f;
	  acc_z = ((float)(accel_data[2]))/100.0f;
	  //acc_x = KalmanFilter_Update(&kf_acc_x, ((float)(accel_data[0]))/100.0f);
	  //acc_y = KalmanFilter_Update(&kf_acc_y, ((float)(accel_data[1]))/100.0f);
	  //acc_z = KalmanFilter_Update(&kf_acc_z, ((float)(accel_data[2]))/100.0f);

	  // Gyroscope
	  GetGyroData(&hi2c1, (uint8_t*)gyro_readings);
	  gyro_data[0] = (((int16_t)gyro_readings[1] << 8) | gyro_readings[0]);
	  gyro_data[1] = (((int16_t)gyro_readings[3] << 8) | gyro_readings[2]);
	  gyro_data[2] = (((int16_t)gyro_readings[5] << 8) | gyro_readings[4]);
	  gyr_x = (((float)(gyro_data[0]))/16.0f)* (M_PI / 180.0f);  // rad/s değil, deg/s cinsinden (datasheet'e göre)
	  gyr_y = (((float)(gyro_data[1]))/16.0f)* (M_PI / 180.0f);
	  gyr_z = (((float)(gyro_data[2]))/16.0f)* (M_PI / 180.0f);

	  float accel[3] = {acc_x, acc_y, acc_z};
	  float gyro[3]  = {gyr_x, gyr_y, gyr_z};


	  Mahony_UpdateIMU(&ahrs, gyro, accel);
	  quatern2rotMat(ahrs.Quaternion, (float (*)[3])R, 1);

	  float acc_body[3] = {acc_x, acc_y, acc_z};
	  ComputeTiltCompensatedAcc((float (*)[3])R, acc_body, tcAcc);


	  linAcc[0] = tcAcc[0] * 9.81f;
	  linAcc[1] = tcAcc[1] * 9.81f;
	  linAcc[2] = (tcAcc[2] - 1.0f) * 9.81f;

	  LinVel_Update(linAcc, linVel, samplePeriod);
	  LinPos_Update(linVel,linPos,samplePeriod);

      gyr_x = KalmanFilter_Update(&kf_gyr_x, (((float)(gyro_data[0]))/16.0f)* (M_PI / 180.0f));
      gyr_y = KalmanFilter_Update(&kf_gyr_y, (((float)(gyro_data[1]))/16.0f)* (M_PI / 180.0f));
      gyr_z = KalmanFilter_Update(&kf_gyr_z, (((float)(gyro_data[2]))/16.0f)* (M_PI / 180.0f));
	  printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n",
	           acc_x, acc_y, acc_z,
	           gyr_x, gyr_y, gyr_z,
	           R[0][0], R[0][1], R[0][2],
	           R[1][0], R[1][1], R[1][2],
	           R[2][0], R[2][1], R[2][2],
	           tcAcc[0], tcAcc[1], tcAcc[2],
	           linAcc[0], linAcc[1], linAcc[2],
			   linVel[0], linVel[1], linVel[2],
			   linPos[0], linPos[1], linPos[2]);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */


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
  __HAL_RCC_GPIOA_CLK_ENABLE();
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
