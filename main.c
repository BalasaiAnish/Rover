/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MPU6050_ADDR 0xD0
#define REG_SMPLRT_DIV 0x19
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_TEMP_OUT_H 0x41
#define REG_GYRO_XOUT_H 0x43
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
static int32_t LeftWheelEncoder=0;
static int32_t RightWheelEncoder=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void subscription_cmd_vel_callback(const void * msgin)
{
	geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *)msgin;

	if (msg->linear.x >= 0) {
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
		TIM3->CCR1 = 200*msg->linear.x;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 200*msg->linear.x;
		TIM3->CCR4 = 0;

	}
	else {
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
		TIM3->CCR1 = 0;
		TIM3->CCR2 = -200*msg->linear.x;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = -200*msg->linear.x;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	 ->Interrupt callback function that modifies position using wheel encoders
	 ->Checks GPIO pin in argument to change left/right wheel position
	 */
	if (GPIO_Pin == GPIO_PIN_0)					//PA0 LeftWheelEncoderChannelA  PE11 LeftWheelEncoderChannelB
	{

		switch(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
		{
		case 0:
			LeftWheelEncoder++;
			break;

		case 1:
			LeftWheelEncoder--;
		}
	}
	else if (GPIO_Pin == GPIO_PIN_1)					//PA1 RightWheelEncoderChannelA  PE12 RightWheelEncoderChannelB
	{

		switch(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12))
				{
				case 0:
					RightWheelEncoder++;
					break;

				case 1:
					RightWheelEncoder--;
	}
}

// Creating a struct to hold IMU readings
struct IMU_Data
{
	float x_acc;
	float y_acc;
	float z_acc;

	float x_gyro;
	float y_gyro;
	float z_gyro;

};

typedef struct IMU_Data IMU_Data;

// Initialising struct and using a pointer

IMU_Data imu_data;
IMU_Data* imu_data = &imu_data;

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  uint8_t check = 0;
  uint8_t* check_ptr = &check;

  uint8_t reg_data = 0;
  uint8_t* reg_data_ptr = &reg_data;

  *checkPtr = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, REG_WHO_AM_I, 1, &Check, 1, 1000);

  if (*check_ptr == 0x68)
  {

    *reg_data_ptr = 0;

    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_PWR_MGMT_1, 1,dataPtr, 1, 1000);

   *reg_data_ptr= 0x07;

   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_SMPLRT_DIV, 1, dataPtr, 1, 1000);

   *reg_data_ptr= 0x00;

   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_ACCEL_CONFIG, 1, dataPtr, 1, 1000);

   *reg_data_ptr = 0x00;

   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_GYRO_CONFIG, 1, dataPtr, 1, 1000);

   }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	  // micro-ROS configuration

	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app

	  rcl_publisher_t publisher1;
	  rcl_publisher_t publisher2;
	  rcl_subscription_t subscriber_cmd_vel;
	  std_msgs__msg__Int32 msg1;
	  std_msgs__msg__Int32 msg2;
	  geometry_msgs__msg__Twist sub_cmd_vel_msg;
	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "stm32_microros", "", &support);

	  // create publisher
	  rclc_publisher_init_default(
			&publisher1,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"left_wheel_encoder");

	  rclc_publisher_init_default(
				&publisher2,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
				"right_wheel_encoder");

	  // create subscriber

	  rclc_subscription_init_default(
				 &subscriber_cmd_vel,
				 &node,
				 ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
				 "cmd_vel");

	  	// create executor
	  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	  rclc_executor_init(&executor, &support.context, 2, &allocator);
	  clc_executor_add_subscription(&executor, &subscriber_cmd_vel, &sub_cmd_vel_msg, &subscription_cmd_vel_callback, ON_NEW_DATA);

	  // Reading Accelerometer data

	  uint8_t acc_data[6];

	  HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, acc_data, 6, 1000);

	  int16_t accel_x_raw = (int16_t)(Rec_Data[0] << 8 | Read_Data [1]);

	  int16_t accel_y_raw = (int16_t)(Rec_Data[2] << 8 | Read_Data [3]);

	  int16_t accel_z_raw = (int16_t)(Rec_Data[4] << 8 | Read_Data [5]);

	  imu_data->x_acc = accel_x_raw / 16384.0;

	  imu_data->y_acc = accel_y_raw / 16384.0;

	  imu_data->z_acc = accel_z_raw / 16384.0;

	  // Reading Gyro data
	  uint8_t gyro_data[6];

	  HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, REG_GYRO_XOUT_H, 1, gyro_data, 6, 1000);

	  int16_t x_gyro_raw = (int16_t)(Rec_Data[0] << 8 | Read_Data [1]);

	  int16_t y_gyro_raw = (int16_t)(Rec_Data[2] << 8 | Read_Data [3]);

	  int16_t z_gyro_raw = (int16_t)(Rec_Data[4] << 8 | Read_Data [5]);



	  imu_data->x_gyro = (x_gyro_raw)/131.0;

	  imu_data->y_gyro = (y_gyro_raw)/131.0;

	  imu_data->z_gyro = (z_gyro_raw)/131.0;


	  // Microros publishing

	  for(;;)
	  {
		msg1.data = left_wheel_encoder;
		msg2.data = right_wheel_encoder;
	    rcl_ret_t ret1 = rcl_publish(&publisher1, &msg1, NULL);
	    rcl_ret_t ret2 = rcl_publish(&publisher2, &msg2, NULL);
	    rclc_executor_spin_some(&executor, 1000);    						// waits for 1000ns for ros data, theres no data it continues, if there is data then it executes subscription callback

	    if ((ret1 | ret2) != RCL_RET_OK)
	    {
	      printf("Error publishing (line %d)\n", __LINE__);
	    }
	    osDelay(10);
	  }

//	  rclc_executor_spin(&executor);
  /* USER CODE END 5 */
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
