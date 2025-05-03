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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    TRIGGER_MODE_RISING,
    TRIGGER_MODE_FALLING,
    TRIGGER_MODE_BOTH
} trigger_mode_enum;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SCREEN_TRIGGERS         4

#define SCREEN_SIZE             200

#define SCREEN_DATA_SIZE		800		   // SCREEN_TRIGGERS * SCREEN_SIZE
#define ADC_INTERMEDIATE_SIZE	2
#define ADC_BUFFER_SIZE			10800	   // ADC_SAMPLING_RATE * ADC_BUFFER_TIME / 1000

#define ADC_SAMPLING_RATE       5400000    // Hz
#define ADC_BUFFER_TIME    		2          // ms
#define ADC_RESOLUTION          8          // bits
#define ADC_VOLTAGE_MAX         3.3        // V
#define ADC_VOLTAGE_MIN         0		   // V

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for captureDataTask */
osThreadId_t captureDataTaskHandle;
const osThreadAttr_t captureDataTask_attributes = {
  .name = "captureDataTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for measureTask */
osThreadId_t measureTaskHandle;
const osThreadAttr_t measureTask_attributes = {
  .name = "measureTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for testTask */
osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

volatile bool start_finding_trigger;     // indicate the task start finding trigger, adc-dma end control
volatile bool screen_data_ready;         // control by the program task, send signal to let the screen side to know ok to read data
volatile bool screen_measure_ready;
bool is_adc_buffer_first_half_active;      // indicate which half of adc buffer is active, adc-dma end control
bool trigger_found;

trigger_mode_enum trigger_mode = TRIGGER_MODE_FALLING;

// const int adc_buffer_size = ADC_SAMPLING_RATE * ADC_BUFFER_HALF_TIME / 1000;
// const int screen_data_size = SCREEN_TRIGGERS * SCREEN_SIZE;
int time_scale = 1;             // capture 1 data from every n points

uint8_t adc_buffer[ADC_BUFFER_SIZE];
uint8_t adc_intermediate[ADC_INTERMEDIATE_SIZE][SCREEN_SIZE] = {0};   // should initialize
int8_t screen_data[SCREEN_DATA_SIZE];

uint8_t* adc_intermediate_start_ptr;
uint8_t* adc_intermediate_end_ptr;		// closed

int adc_intermediate_index = 0;
uint8_t* adc_intermediate_ptr;
int half_screen_size = SCREEN_SIZE / 2;
int half_adc_buffer_size = ADC_BUFFER_SIZE / 2;

uint8_t trigger_level = 128;    // 0-255, 128 is the middle level
uint8_t offset = 128;

uint8_t prev_val;
uint8_t curr_val;

uint8_t* adc_buffer_read_start_ptr;
uint8_t* adc_buffer_read_end_ptr;
uint8_t* adc_buffer_read_ptr;
uint8_t* adc_buffer_after_trigger_read_end_ptr;

uint8_t* adc_buffer_read_find_trigger_start_ptr;
uint8_t* adc_buffer_read_find_trigger_end_ptr;

// screen end know where to read the 200 signed data
uint8_t* screen_data_start_ptr;
uint8_t* screen_frame_start_ptr;
uint8_t* screen_frame_end_ptr;		// closed
uint8_t* screen_measure_ptr;


double v_pp_output;
double frequency_output;
double period_output;

int trigger_crossing_count;
double screen_frame_time;

int measure_loop_index;
uint8_t max_val;
uint8_t min_val;

const uint8_t zero_val = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartCaptureDataTask(void *argument);
void StartMeasureTask(void *argument);
void StartTestTask(void *argument);

/* USER CODE BEGIN PFP */
void captureData(void);
void measure(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void captureData(void)
{
    // screen_data_ready = false;
    // screen_measure_ready = false;
    if (is_adc_buffer_first_half_active)
    {
        adc_buffer_read_start_ptr = &adc_buffer[0];
        adc_buffer_read_end_ptr = &adc_buffer[half_adc_buffer_size];
    }
    else
    {
        adc_buffer_read_start_ptr = &adc_buffer[half_adc_buffer_size];
        adc_buffer_read_end_ptr = &adc_buffer[ADC_BUFFER_SIZE];
    }

    adc_buffer_read_find_trigger_start_ptr = adc_buffer_read_start_ptr +  time_scale * half_screen_size;
    adc_buffer_read_find_trigger_end_ptr = adc_buffer_read_end_ptr - time_scale * half_screen_size;

    adc_intermediate_start_ptr = adc_intermediate[adc_intermediate_index];
    adc_intermediate_end_ptr = adc_intermediate_start_ptr + SCREEN_SIZE - 1;

    screen_frame_start_ptr = adc_intermediate_start_ptr;
    screen_frame_end_ptr = adc_intermediate_end_ptr;

    adc_intermediate_ptr = adc_intermediate_start_ptr;
    adc_buffer_read_ptr = adc_buffer_read_start_ptr;

    curr_val = *adc_buffer_read_ptr;
    prev_val = curr_val;
    *adc_intermediate_ptr = prev_val;
    adc_intermediate_ptr++;
    adc_buffer_read_ptr += time_scale;

    trigger_found = false;

    while (adc_buffer_read_ptr < adc_buffer_read_end_ptr)
    {
    	if (adc_buffer_read_ptr < adc_buffer_read_find_trigger_start_ptr)
    	{
    		curr_val = *adc_buffer_read_ptr;
    		*adc_intermediate_ptr = curr_val;
    		prev_val = curr_val;
    		adc_intermediate_ptr = adc_intermediate_ptr == adc_intermediate_end_ptr ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;
    		adc_buffer_read_ptr += time_scale;
    		continue;
    	}

        curr_val = *adc_buffer_read_ptr;
        *adc_intermediate_ptr = curr_val;


        switch (trigger_mode)
        {
            case TRIGGER_MODE_RISING :
                trigger_found = (prev_val <= trigger_level && curr_val > trigger_level);
                break;
            case TRIGGER_MODE_FALLING :
                trigger_found = (prev_val >= trigger_level && curr_val < trigger_level);
                break;
            case TRIGGER_MODE_BOTH :
                trigger_found = (prev_val <= trigger_level && curr_val > trigger_level) || (prev_val >= trigger_level && curr_val < trigger_level);
                break;
        }

        if (trigger_found)
        {
            // screen_data_start_ptr = adc_intermediate_ptr;

            adc_buffer_after_trigger_read_end_ptr = adc_buffer_read_ptr + (time_scale * half_screen_size);

            adc_buffer_read_ptr = adc_buffer_read_ptr + time_scale;

            adc_intermediate_ptr = (adc_intermediate_ptr == adc_intermediate_end_ptr) ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;

            for (; adc_buffer_read_ptr < adc_buffer_after_trigger_read_end_ptr; adc_buffer_read_ptr += time_scale)
            {
                *adc_intermediate_ptr = *adc_buffer_read_ptr;
                adc_intermediate_ptr = (adc_intermediate_ptr == adc_intermediate_end_ptr) ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;
            }

            screen_data_start_ptr = adc_intermediate_ptr;
            screen_data_ready = true;
            break;
        }

        prev_val = curr_val;
        adc_intermediate_ptr = adc_intermediate_ptr == adc_intermediate_end_ptr ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;
        adc_buffer_read_ptr += time_scale;
    }

    if (!trigger_found)
    {
    	// fail to find any trigger in this half of adc buffer
    }

    // toggle adc_intermediate_index
}


void measure(void)
{
    trigger_crossing_count = 0;
    screen_measure_ptr = screen_data_start_ptr;
    prev_val = *screen_measure_ptr;
    max_val = prev_val;
    min_val = prev_val;
    screen_measure_ptr = (screen_measure_ptr == screen_frame_end_ptr) ? screen_frame_start_ptr : screen_measure_ptr + 1;

    for (measure_loop_index = 1; measure_loop_index < SCREEN_SIZE; measure_loop_index++)
    {
        curr_val = *screen_measure_ptr;
        if (((prev_val >= trigger_level) && (curr_val < trigger_level)) || ((prev_val <= trigger_level) && (curr_val > trigger_level)))
        {
            trigger_crossing_count++;
        }

        max_val = (curr_val > max_val) ? curr_val : max_val;
        min_val = (curr_val < min_val) ? curr_val : min_val;

        prev_val = curr_val;
        screen_measure_ptr = (screen_measure_ptr == screen_frame_end_ptr) ? screen_frame_start_ptr : screen_measure_ptr + 1;
    }

    v_pp_output = (double)(max_val - min_val) * (ADC_VOLTAGE_MAX - ADC_VOLTAGE_MIN) / 255.0;
    frequency_output = (double)(trigger_crossing_count * ADC_SAMPLING_RATE) / (2 * SCREEN_SIZE * time_scale);
    period_output = 1.0 / frequency_output;

    // screen_measure_ready = true;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  // MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

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

  /* creation of captureDataTask */
  captureDataTaskHandle = osThreadNew(StartCaptureDataTask, NULL, &captureDataTask_attributes);

  /* creation of measureTask */
  measureTaskHandle = osThreadNew(StartMeasureTask, NULL, &measureTask_attributes);

  /* creation of testTask */
  testTaskHandle = osThreadNew(StartTestTask, NULL, &testTask_attributes);

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
  // Generate_Sine_Table();
  // index = 0;
  // HAL_TIM_Base_Start_IT(&htim2);

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 536870911;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB8 PB4 PB9 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA15 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PI3 PI2 PI1 PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG7 PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PH6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCaptureDataTask */
/**
* @brief Function implementing the captureDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCaptureDataTask */
void StartCaptureDataTask(void *argument)
{
  /* USER CODE BEGIN StartCaptureDataTask */
  /* Infinite loop */
  for(;;)
  {
	  if (start_finding_trigger)
	  {
		  start_finding_trigger = false;
		  screen_data_ready = false;
		  trigger_found = false;

		  captureData();

		  adc_intermediate_index = (adc_intermediate_index == ADC_INTERMEDIATE_SIZE - 1) ? 0 : adc_intermediate_index + 1;

	  }
	  vTaskDelay(1);
  }
  /* USER CODE END StartCaptureDataTask */
}

/* USER CODE BEGIN Header_StartMeasureTask */
/**
* @brief Function implementing the measureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMeasureTask */
void StartMeasureTask(void *argument)
{
  /* USER CODE BEGIN StartMeasureTask */
  /* Infinite loop */
  for(;;)
  {
	  if (screen_data_ready)
	  {
		  screen_measure_ready = false;
		  measure();
		  screen_measure_ready = true;
	  }
    vTaskDelay(1);
  }
  /* USER CODE END StartMeasureTask */
}

/* USER CODE BEGIN Header_StartTestTask */
/**
* @brief Function implementing the testTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestTask */
void StartTestTask(void *argument)
{
  /* USER CODE BEGIN StartTestTask */
  const double test_freq = 1700000.0;
  const double sample_rate = (double)ADC_SAMPLING_RATE;
  const double amplitude = 64.0;
  const double test_offset = 128.0;

  double t, value;
  for (int i = 0; i < ADC_BUFFER_SIZE; ++i)
  {
	  t = (double)i / sample_rate;
	  value = amplitude * sin(2 * M_PI * test_freq * t) + test_offset;
	  if (value < 0)	value = 0;
	  if (value > 255)	value = 255;
	  adc_buffer[i] = (uint8_t)(value + 0.5);
  }

  is_adc_buffer_first_half_active = true;
  time_scale = 1;

  screen_data_ready = false;
  screen_measure_ready = false;

  start_finding_trigger = true;
  /* Infinite loop */
  while (!screen_measure_ready)
  {
	  osDelay(10);
  }

  if (screen_measure_ready)
  {
	  printf("Max: %d, Min: %d, Vpp: %.2f V, Freq: %.2f Hz, Period: %.6f s\n", max_val, min_val, v_pp_output, frequency_output, period_output);
  }

  vTaskSuspend(NULL);
  /* USER CODE END StartTestTask */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
