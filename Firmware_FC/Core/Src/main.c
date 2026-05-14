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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */

uint8_t incoming; // Speicher für das Byte vom Nano

struct DataPacket {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t arm;
    int16_t mode;
} dataPacket;

char rx_data;          // Einzelnes empfangenes Zeichen
char line_buffer[64];  // Speicher für die komplette Zeile

int buffer_index = 0;


uint8_t buffer[14];
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;

float gyro_z_ds;

uint32_t debug_counter = 0;


float accel_pitch, accel_roll;
float filtered_pitch, filtered_roll;
float gyro_x_ds, gyro_y_ds; // Grad pro Sekunde für X und Y

float dt = 0.01f; // 10ms Loop-Zeit (da HAL_Delay(10) am Ende steht)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  uint8_t power_mgmt = 0;
  HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x6B, 1, &power_mgmt, 1, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_UART_Receive(&huart1, &incoming, 1, 10) == HAL_OK)
	  	  	      {
	  	  	          // Jedes Zeichen im Buffer sammeln (außer das Zeilenende)
	  	  	          if (incoming != '\n' && incoming != '\r')
	  	  	          {
	  	  	              if (buffer_index < 63) {
	  	  	                  line_buffer[buffer_index++] = incoming;
	  	  	              }
	  	  	          }

	  	  	          if (incoming == '\n')
	  	  	          {
	  	  	              line_buffer[buffer_index] = '\0'; // String Ende markieren

	  	  	              // DEIN CODE (unverändert in der Logik):
	  	  	              //printf("Control Values: %s\r\n", line_buffer);
	  	  	              HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  	  	              // Buffer für das nächste Paket leeren
	  	  	              buffer_index = 0;
	  	  	          }
	  	  	      }

	  if (HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, buffer, 14, 10) == HAL_OK)
	      {
	          // 1. Rohdaten extrahieren
	          accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
	          accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
	          accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);
	          gyro_x  = (int16_t)(buffer[8] << 8 | buffer[9]);
	          gyro_y  = (int16_t)(buffer[10] << 8 | buffer[11]);
	          gyro_z  = (int16_t)(buffer[12] << 8 | buffer[13]);

	          // 2. Umrechnen in physikalische Einheiten
	          gyro_x_ds = gyro_x / 131.0f;
	          gyro_y_ds = gyro_y / 131.0f;
	          gyro_z_ds = gyro_z / 131.0f;

	          // 3. Winkel aus Beschleunigungssensor (Referenz für "unten")
	          //atan2 liefert Bogenmaß, daher * 180 / PI
	          accel_pitch = atan2f((float)accel_y, (float)accel_z) * 57.295f;
	          accel_roll  = atan2f(-(float)accel_x, sqrtf((float)accel_y * accel_y + (float)accel_z * accel_z)) * 57.295f;

	          // 4. DER KOMPLEMENTÄRFILTER
	          // 98% Vertrauen in das integrierte Gyro-Signal, 2% Korrektur durch Accel
	          filtered_pitch = 0.99f * (filtered_pitch + gyro_x_ds * dt) + 0.01f * accel_pitch;
	          filtered_roll  = 0.99f * (filtered_roll  + gyro_y_ds * dt) + 0.01f * accel_roll;

	          // Yaw hat keine absolute Referenz (außer Kompass), daher nur Integration + Drift-Risiko
	          // Oder wir nutzen hier nur die Rate (gyro_z_ds) für den PID, wie besprochen.

	          // --- Debug Print ---
	          debug_counter++;
	          if (debug_counter >= 2)
	          {
	        	  printf("IMU:%.2f,%.2f,%.2f\n", filtered_pitch, filtered_roll, gyro_z_ds);
	              debug_counter = 0;
	          }
	      }

	      HAL_Delay(10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
