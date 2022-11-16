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
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;

RTC_DateTypeDef sDate;
RTC_TimeTypeDef sTime;

uint8_t Flag_TIM7;
uint8_t Flag_EXTI15;


const float RAIN_INC_MM = 0.2794;					//Height of precipitation for a bucket in mm
const int HOUR_SECONDS = 3600;
const int DAY_SECONDS = 24* HOUR_SECONDS;
const int WEEK_SECONDS = 7* DAY_SECONDS;
const int MONTH_SECONDS;
uint32_t timestamp;
uint32_t rain_events[1000];
float rain_hourly = 0;
float rain_daily = 0;
float rain_weekly = 0;
float rain_monthly = 0;
uint16_t rain_events_size = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file ,char*ptr,int len){
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ptr, len);
	return len;
}


int epoch_days_fast(int y, int m, int d) {
  const uint32_t year_base = 4800;
  const uint32_t m_adj = m - 3;
  const uint32_t carry = m_adj > m ? 1 : 0;
  const uint32_t adjust = carry ? 12 : 0;
  const uint32_t y_adj = y + year_base - carry;
  const uint32_t month_days = ((m_adj + adjust) * 62719 + 769) / 2048;
  const uint32_t leap_days = y_adj / 4 - y_adj / 100 + y_adj / 400;
  return y_adj * 365 + leap_days + month_days + (d - 1) - 2472632;
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
  MX_TIM7_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  printf("\n%s\n\r","Hello Putty.");

	RTC_SetDate(&sDate, 22, 11, 9, 2);
	RTC_SetTime(&sTime, 11, 00, 00);

  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
  	if (Flag_EXTI15 == 1){
  		HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
			/* Get the RTC current Date */
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			timestamp = epoch_days_fast(sDate.Year+2000, sDate.Month, sDate.Date)*DAY_SECONDS+ (sTime.Hours*3600+sTime.Minutes*60+sTime.Seconds);
			printf("Date : %02u:%02u:%04u ",sDate.Date, sDate.Month, 2000 + sDate.Year);
			HAL_Delay(5);
			printf("@ %02u:%02u:%02u\n\r",sTime.Hours, sTime.Minutes, sTime.Seconds);
			HAL_Delay(5);
			printf("Timestamp : %lu\n\r",timestamp);
			HAL_Delay(5);


			printf("%d Rain events.\n\r",rain_events_size + 1);
			HAL_Delay(5);
			rain_events[rain_events_size] = timestamp;
			rain_events_size ++;
			for (uint16_t i = 0; i< rain_events_size; i++){
				if (rain_events[i] >= timestamp - MONTH_SECONDS){
					rain_hourly+=RAIN_INC_MM;
					if (rain_events[i] >= timestamp - WEEK_SECONDS){
						rain_daily+=RAIN_INC_MM;
						if (rain_events[i] >= timestamp - DAY_SECONDS){
							rain_weekly+=RAIN_INC_MM;
							if (rain_events[i] >= timestamp - HOUR_SECONDS){
								rain_monthly+=RAIN_INC_MM;
							}
						}
					}
				}
				else rain_events[i] = 0;
			}
			//printf("Rain h %d, d %d, w %d, m %d\n\r",rain_hourly,rain_daily,rain_weekly,rain_monthly);
			//HAL_Delay(5);
			printf("--------------------------------\n\r");
			Flag_EXTI15 = 0;
		}
		else if (Flag_TIM7 == 1){
			//500 mHz blink
			//HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
			Flag_TIM7 = 0;
		}
		else{
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_ResumeTick();
	if(GPIO_Pin == RAIN_Pin){
		Flag_EXTI15 = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	HAL_ResumeTick();
	if (htim == &htim7){
		Flag_TIM7 = 1 ;
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
