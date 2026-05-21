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
#include "tim.h"
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

// -- Define Variables for RF Connection --

uint8_t incoming;	// Stores incoming data from Receiver Bridge

char line_buffer[64];	// Stores complete line from Receiver Bridge
int buffer_index = 0;	// Index in line_buffer

struct DataPacket {  // datatype for controll values
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t arm;
    int16_t mode;
}controlInput;

// -- Define Variables for reading IMU --

uint8_t buffer[14];	// Stores incoming data from IMU
int16_t accel_x, accel_y, accel_z;	// Raw Data
int16_t gyro_x, gyro_y, gyro_z;	// Raw Data

float gyro_x_ds, gyro_y_ds, gyro_z_ds;	// Gyro rate of change per second
float accel_pitch, accel_roll;	// Calibrated angles
float raw_accel_pitch, raw_accel_roll;	// Calculated angles
float filtered_pitch, filtered_roll;	// Filtered angles

float dt = 0.01f; // Duration of loop for angle integration

float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;	// Variables for IMU calibration
float accel_pitch_offset = 0, accel_roll_offset = 0;

// -- Define PID Variables --

struct PID_Config {
	float kp;
	float ki;
	float kd;
	float integral;
	float lastError;
};

// Tuning values for coefficients
struct PID_Config pid_pitch = {1.2f, 0.05f, 0.1f, 0.0f, 0.0f};
struct PID_Config pid_roll  = {1.2f, 0.05f, 0.1f, 0.0f, 0.0f};
struct PID_Config pid_yaw   = {2.0f, 0.02f, 0.0f, 0.0f, 0.0f};

// Output values for ESC
volatile float pid_out_pitch = 0.0f;
volatile float pid_out_roll  = 0.0f;
volatile float pid_out_yaw   = 0.0f;


// -- Define Debugging Variables --

uint32_t debug_counter = 0;	// Determine debugging output rate

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Read_RF_Receiver(void);
HAL_StatusTypeDef Read_IMU(void);
void Calculate_PID(void);
void Calibrate_IMU(void);

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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Initialize timer interrupt
  HAL_TIM_Base_Start_IT(&htim6);

  // Initialize IMU
  uint8_t power_mgmt = 0;
  HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1), 0x6B, 1, &power_mgmt, 1, 100);
  HAL_Delay(100);

  // Calibrate IMU
  Calibrate_IMU();





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
	  // Read data from receiver bridge
	  Read_RF_Receiver();

	  // -- Debug Print (enable only when debugging!) --
	  if (debug_counter >= 1)
	  {
		  //printf(line_buffer);
	  	  printf("IMU:%.2f,%.2f,%.2f\n", filtered_pitch, filtered_roll, gyro_z_ds);

	  	  debug_counter = 0;
	  }

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

/**
  * @brief  Timer interrupt executed every 10 ms for IMU reading and PID calculation
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Check if tim6 caused interrupt
    if (htim->Instance == TIM6)
    {
        // Read IMU data
        if (Read_IMU() == HAL_OK)
        {
            // Calculate PID value for motor control
            Calculate_PID();

            debug_counter++;
        }
    }
}


/**
  * @brief  Calibrates IMU before taking flight
  * @retval None
  */
void Calibrate_IMU(void)
{
	// --- Start calibration ---
	printf("Calibrating IMU... Do not move Drone!\r\n");

	float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
	float pitch_sum = 0, roll_sum = 0;
	int samples = 500;

	for (int i = 0; i < samples; i++)
	{
	    if (HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, buffer, 14, 10) == HAL_OK)
	    {
	        // Extract raw data from IMU
	        int16_t ax = (int16_t)(buffer[0] << 8 | buffer[1]);
	        int16_t ay = (int16_t)(buffer[2] << 8 | buffer[3]);
	        int16_t az = (int16_t)(buffer[4] << 8 | buffer[5]);
	        int16_t gx = (int16_t)(buffer[8] << 8 | buffer[9]);
	        int16_t gy = (int16_t)(buffer[10] << 8 | buffer[11]);
	        int16_t gz = (int16_t)(buffer[12] << 8 | buffer[13]);

	        // Sum gyro rates
	        gyro_x_sum += gx / 131.0f;
	        gyro_y_sum += gy / 131.0f;
	        gyro_z_sum += gz / 131.0f;

	        // Calculate uncorrected accelerometer angles for test
	        float raw_pitch = atan2f((float)ay, (float)az) * 57.295f;
	        float raw_roll  = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * 57.295f;

	        pitch_sum += raw_pitch;
	        roll_sum += raw_roll;
	    }
	    HAL_Delay(2);
	}

	// Calculate average error
	gyro_x_offset = gyro_x_sum / samples;
	gyro_y_offset = gyro_y_sum / samples;
	gyro_z_offset = gyro_z_sum / samples;

	accel_pitch_offset = pitch_sum / samples;
	accel_roll_offset  = roll_sum / samples;

	printf("Calibration done! P_off: %.2f | R_off: %.2f\r\n", accel_pitch_offset, accel_roll_offset);
	// --- End of calibration ---
}


/**
  * @brief  Reads control values from receiver bridge
  * @retval None
  */
void Read_RF_Receiver(void)
{
	// -- Receive data from receiver bridge --
	if(HAL_UART_Receive(&huart1, &incoming, 1, 0) == HAL_OK)
	{
		// Collect data in line_buffer exept for end of line
		if (incoming != '\n' && incoming != '\r')
	  	{
			if (buffer_index < 63)	// Check for space in line_buffer
			{
				line_buffer[buffer_index++] = incoming;
	  	  	}
	  	}

		// Detect end of line
	  	if (incoming == '\n')
	  	{
	  		line_buffer[buffer_index] = '\0'; // Mark end of string
	  	  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);	// Toggle led to showcase received data

	  	  	// Parsing to struct
	  	  	sscanf(line_buffer, "%hd,%hd,%hd,%hd,%hd",
	  	  	&controlInput.pitch,
			&controlInput.roll,
			&controlInput.yaw,
			&controlInput.arm,
			&controlInput.mode);

	  	  	buffer_index = 0;	// Reset buffer_index for next incoming data
	  	}
	}
}


/**
  * @brief  Reads data from IMU
  * @retval Enum indicating success of reading IMU
  */
HAL_StatusTypeDef Read_IMU(void)
{
	// -- Receive data from IMU --
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3B, 1, buffer, 14, 5);

    if (status == HAL_OK)
    {
    	// Extract raw data from IMU
		accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
	    accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
	    accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);
	    gyro_x  = (int16_t)(buffer[8] << 8 | buffer[9]);
	    gyro_y  = (int16_t)(buffer[10] << 8 | buffer[11]);
	    gyro_z  = (int16_t)(buffer[12] << 8 | buffer[13]);

	    // Convert raw values to physical unit -> degrees/second
	    gyro_x_ds = (gyro_x / 131.0f) - gyro_x_offset;
	    gyro_y_ds = (gyro_y / 131.0f) - gyro_y_offset;
	    gyro_z_ds = (gyro_z / 131.0f) - gyro_z_offset;

	    // Calculate angle of pitch and roll from accelerometer and convert to degrees
	    raw_accel_pitch = atan2f((float)accel_y, (float)accel_z) * 57.295f;
	    raw_accel_roll  = atan2f(-(float)accel_x, sqrtf((float)accel_y * accel_y + (float)accel_z * accel_z)) * 57.295f;

	    // Correct angle of pitch and roll with calibrated offset
	    accel_pitch = raw_accel_pitch - accel_pitch_offset;
	    accel_roll = raw_accel_roll - accel_roll_offset;

	    // Filter/Fuse values with complementary filter (tune here)
	    filtered_pitch = 0.996f * (filtered_pitch + gyro_x_ds * dt) + 0.004f * accel_pitch;
	    filtered_roll  = 0.996f * (filtered_roll  + gyro_y_ds * dt) + 0.004f * accel_roll;

	    // No absolute value for yaw, only rate of change from gyro is used
	    // due to risk of integration error
    }
    return status;
}


/**
  * @brief  Calculates outputs for ESC using PID control
  * @retval None
  */
void Calculate_PID(void)
{
    // --- Pitch ---
    float error_pitch = (float)controlInput.pitch - filtered_pitch;
    float p_term_pitch = pid_pitch.kp * error_pitch;

    pid_pitch.integral += error_pitch * dt;
    if (pid_pitch.integral > 400.0f)  pid_pitch.integral = 400.0f; // Anti-Windup
    if (pid_pitch.integral < -400.0f) pid_pitch.integral = -400.0f;

    float d_term_pitch = pid_pitch.kd * (error_pitch - pid_pitch.lastError) / dt;
    pid_out_pitch = p_term_pitch + (pid_pitch.ki * pid_pitch.integral) + d_term_pitch;
    pid_pitch.lastError = error_pitch;

    // --- Roll ---
    float error_roll = (float)controlInput.roll - filtered_roll;
    float p_term_roll = pid_roll.kp * error_roll;

    pid_roll.integral += error_roll * dt;
    if (pid_roll.integral > 400.0f)  pid_roll.integral = 400.0f; // Anti-Windup
    if (pid_roll.integral < -400.0f) pid_roll.integral = -400.0f;

    float d_term_roll = pid_roll.kd * (error_roll - pid_roll.lastError) / dt;
    pid_out_roll = p_term_roll + (pid_roll.ki * pid_roll.integral) + d_term_roll;
    pid_roll.lastError = error_roll;

    // --- YAW ---
    float error_yaw = (float)controlInput.yaw - gyro_z_ds;
    float p_term_yaw = pid_yaw.kp * error_yaw;

    pid_yaw.integral += error_yaw * dt;
    if (pid_yaw.integral > 200.0f)  pid_yaw.integral = 200.0f;
    if (pid_yaw.integral < -200.0f) pid_yaw.integral = -200.0f;

    pid_out_yaw = p_term_yaw + (pid_yaw.ki * pid_yaw.integral);
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
