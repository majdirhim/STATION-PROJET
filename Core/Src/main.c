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
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "Carte_Sd.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_lcd.h"
#include "FontRoboto16.h"
#include "FontRoboto20.h"
#include "FontRoboto24.h"
#include "FontRoboto32.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/**************Vitesse Du Vent***************/
#define NO_WIND 0.3
#define MPH_CONST 1.492
#define KMH_CONST 1.609 // 1 MPH = 1.609 KM/h

/**************Vitesse Du Vent***************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/**************TYPEDEFS***************/
RTC_DateTypeDef sDate;
RTC_TimeTypeDef sTime;
TS_StateTypeDef TS_State;
/**************TYPEDEFS***************/

/**************FLAGS***************/
uint8_t Flag_TIM7;
uint8_t Flag_EXTI15_RAIN;
uint8_t Flag_EXTI15_TOUCH;
volatile uint8_t Flag_RTCIAA;
/**************FLAGS***************/

/**************LCD TOUCH / DISPLAY***************/
int touch_x;
int touch_y;
uint8_t touched = 0;
/**************LCD TOUCH / DISPLAY***************/

/**************RAINFALL***************/
const float RAIN_INC_MM = 0.2794;	//Height of precipitation for a bucket in mm
const int HOUR_SECONDS = 3600;
const int DAY_SECONDS = 24 * HOUR_SECONDS;
const int WEEK_SECONDS = 7 * DAY_SECONDS;
const int MONTH_SECONDS = 30 * DAY_SECONDS;
uint32_t timestamp;
uint32_t rain_events[30000];
float rain_hourly = 0;
float rain_daily = 0;
float rain_weekly = 0;
float rain_monthly = 0;
float rain_hours[10];
float rain_days[10];
float rain_weeks[10];
float rain_months[10];

uint16_t hour_counter = 0;
uint16_t day_counter = 0;
uint16_t week_counter = 0;
uint16_t month_counter = 0;
uint16_t rain_events_size = 0;
/**************RAINFALL***************/

/**************Vitesse Du Vent***************/
volatile uint8_t TIM1_IC_IT_Flag = 0, FIRST_IMP = 1,i=0;
volatile uint32_t ccr0 = 0, ccr1 = 0;
float Average_Wind_Speed_MPH;
float Average_Wind_Speed_KMH;
/**************Vitesse Du Vent***************/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/**************Vitesse Du Vent***************/
void WSpeed_To_WForce(float Wind_Speed, uint8_t *Force);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
/**************Vitesse Du Vent***************/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, 100);
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

extern void RTC_SetDate(RTC_DateTypeDef * sDate, uint8_t year, uint8_t month, uint8_t date, uint8_t wday);
extern void RTC_SetTime(RTC_TimeTypeDef * sTime,uint8_t hour, uint8_t min, uint8_t sec);

void remove_rain_event(unsigned int idx) {
	for (uint16_t i = idx; i < rain_events_size; i++) {
		rain_events[i] = rain_events[i + 1];
	}
	rain_events_size--;
}

void remove_array_index(float* arr, uint16_t idx, uint16_t * elem_count){
    for (unsigned int i = idx; i <10; i++){
        arr[i] = arr[i+1];
    }
    (*elem_count)--;
    printf("Remove index %d, resulting count : %d\n\r", idx, *elem_count);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/**************Vitesse Du Vent***************/
	uint8_t First_Speed = 1, Force = 0,LastForce=0;
	uint16_t Speed_Sum=0 , W_nb =0;
	float Wind_Speed = 0.0, Wind_Speed_KMH = 0.0, Max_Wind = 0.0,
			Min_Wind = 0.0, Frequency = 0.0 , Average_Wind_Speed=0.0;
	float Tim1_Freq;
	/**************Vitesse Du Vent***************/
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
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  /* USER CODE BEGIN 2 */
  /**************SD Card***********************/
  //Fat_Init(); //Bloque l'exécution
  char wtext[200];
  char rainSD[200];

  /**************SD Card***********************/

  /**************LCD TOUCH / DISPLAY***************/
  BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, SDRAM_DEVICE_ADDR);
	BSP_LCD_SetLayerVisible(LTDC_ACTIVE_LAYER, ENABLE);
	BSP_LCD_SetFont(&LCD_FONT_24);
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
	BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"2 Dec", CENTER_MODE);
	BSP_LCD_SetFont(&LCD_FONT_20);
	BSP_LCD_DisplayStringAt(0, 40, (uint8_t *)"14:00", CENTER_MODE);

	BSP_TS_Init(480, 272);
	BSP_TS_ITConfig();
	/**************LCD TOUCH / DISPLAY***************/

	/**************Pluviométrie******************/
	printf("\nHello Putty.\n\r");

	RTC_SetDate(&sDate, 22, 11, 9, 2);
	RTC_SetTime(&sTime, 11, 00, 00);

	//HAL_TIM_Base_Start_IT(&htim7);
	/**************Pluviométrie******************/

	/**************Vitesse Du Vent***************/
	Tim1_Freq = HAL_RCC_GetPCLK2Freq() / TIM1->PSC; //APB2_PSC=1 et TIM_psc=5000-1
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	/**************Vitesse Du Vent***************/
		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
	while (1) {

		/**************RAINFALL******************/
		if (Flag_EXTI15_RAIN == 1) {
			/* Get the RTC current Date */
			HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			timestamp = epoch_days_fast(sDate.Year + 2000, sDate.Month,
					sDate.Date) * DAY_SECONDS
			+ (sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds);
			printf("Date : %02u:%02u:%04u ", sDate.Date, sDate.Month,
					2000 + sDate.Year);
			printf("@ %02u:%02u:%02u\n\r", sTime.Hours, sTime.Minutes,
					sTime.Seconds);
			printf("Timestamp : %lu\n\r", timestamp);

			printf("%d Rain events.\n\r", rain_events_size + 1);
			rain_events[rain_events_size] = timestamp;
			rain_events_size++;
			for (uint16_t i = 0; i < rain_events_size; i++) {
				if (rain_events[i] >= timestamp - MONTH_SECONDS) {
					rain_hourly += RAIN_INC_MM;
					if (rain_events[i] >= timestamp - WEEK_SECONDS) {
						rain_daily += RAIN_INC_MM;
						if (rain_events[i] >= timestamp - DAY_SECONDS) {
							rain_weekly += RAIN_INC_MM;
							if (rain_events[i] >= timestamp - HOUR_SECONDS) {
								rain_monthly += RAIN_INC_MM;
							}
						}
					}
				} else
				//remove_array_index(rain_events, i, &rain_events_size);
				remove_rain_event(i);
			}
			printf("Rain h %.2fmm, %.2fmm, w %.2fmm, m %.2fmm \n\r",
					rain_hourly, rain_daily, rain_weekly, rain_monthly);
			if(Flag_RTCIAA==1){
				sprintf(rainSD,"Rain h %.2fmm, %.2fmm, w %.2fmm, m %.2fmm \n\r",
								rain_hourly, rain_daily, rain_weekly, rain_monthly);
				WR_TO_Sd(rainSD, "Rain.txt");
			}
			printf("--------------------------------\n\r");
			Flag_EXTI15_RAIN = 0;
		} /*else if (Flag_TIM7 == 1) {
			//500 mHz blink
			//HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
			Flag_TIM7 = 0;
		} else {
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		}*/
		/**************RAINFALL******************/

		/**************LCD TOUCH***************/
		if(Flag_EXTI15_TOUCH == 1){

			BSP_TS_GetState(&TS_State);
				if(TS_State.touchDetected)
				{
					/* Get X and Y position of the touch post calibrated */
					touch_x = TS_State.touchX[0];
					touch_y = TS_State.touchY[0];
					touched = 1;
					printf("Oh you touched my screen (x : %d, y : %d)\n\r",touch_x ,touch_y );
					Flag_EXTI15_TOUCH = 0;
				}
		}
		/**************LCD TOUCH***************/
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
		/**************Vitesse Du Vent***************/
		if (TIM1_IC_IT_Flag) {
			// Calcul de la fréquence dans les deux cas => Avant timer overflow : juste après timer le overflow
			Frequency = ccr1 >= ccr0 ?(float) Tim1_Freq / (ccr1 - ccr0) :(float) Tim1_Freq / ((TIM1->ARR + ccr1) - ccr0);
			// La vitesse du vent(en Mph) correspond à la fréqunce du signal capturée multipliée par une constante
			Wind_Speed = MPH_CONST * Frequency;
			if (First_Speed) {
				//Initialiser les Valeur Max et Min de la vitesse du vent(Mph)
				Min_Wind = Wind_Speed;
				Max_Wind = Wind_Speed;
				First_Speed = 0;
			}
			//CCR1 devient CCR0 pour la prochaine détection d'impulsion
			ccr0 = ccr1;
			//Si la vitesse est négligeable Wind_Speed = 0
			Wind_Speed = Wind_Speed > NO_WIND ? Wind_Speed : 0.0;
			//Convertir la vitesse en Km/h
			Wind_Speed_KMH = KMH_CONST * Wind_Speed;
			// calcul de la maximum et la minimum de la vitesse du vent
			if (Wind_Speed > Max_Wind)
			Max_Wind = Wind_Speed;
			First_Speed = 0;
		}
		//CCR1 devient CCR0 pour la prochaine détection d'impulsion
		ccr0 = ccr1;
		//Si la vitesse est négligeable Wind_Speed = 0
		Wind_Speed = Wind_Speed > NO_WIND ? Wind_Speed : 0.0;
		//Convertir la vitesse en Km/h
		Wind_Speed_KMH = KMH_CONST * Wind_Speed;
		// calcul de la maximum et la minimum de la vitesse du vent
		if (Wind_Speed > Max_Wind)
		Max_Wind = Wind_Speed;
		else if (Wind_Speed < Min_Wind && Wind_Speed != 0)
		Min_Wind = Wind_Speed;
		// calculer la somme des vitesses ( à diviser après par nb pour déterminer la moyenne )
		Speed_Sum+=Wind_Speed ;++W_nb;
		//Force du vent selon l'échelle de Beaufort(à interpréter par une disignation dans l'affichage)
		WSpeed_To_WForce(Wind_Speed, &Force);
		//Envoie à travers le Port Série la vitesse du vent actuelle
		printf("Wind_Speed = %.3f Mph %.3f km/h Min=%.3f Max=%.3f Force =%u\n\r",
				Wind_Speed, Wind_Speed_KMH, Min_Wind, Max_Wind, Force);
		//stocker les données chaque changement de Force de Beaufort
		if(LastForce!=Force){
			LastForce=Force;
			Average_Wind_Speed=(float)Speed_Sum/W_nb;
			sprintf(wtext,"Average_Wind_Speed = %.3f Mph %.3f km/h Min=%.3f Max=%.3f Force =%u\n\r",
					Average_Wind_Speed, Average_Wind_Speed*KMH_CONST, Min_Wind, Max_Wind, Force);
			WR_TO_Sd(wtext, "Wind.txt"); //ecriture dans le fichier wind.txt
			Average_Wind_Speed=0 ;W_nb =0;Speed_Sum=0;
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//Hourly alarm to save data
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	HAL_ResumeTick();
	Flag_RTCIAA = 1;
	HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN);
	printf("%s : %d/%d [%dh]\n\r", "Hourly alarm", sDate.Date, sDate.Month, sTime.Hours);
	rain_hours[hour_counter++] = rain_hourly;
	if (hour_counter == 9) remove_array_index(rain_hours, 0, &hour_counter);
	if (sTime.Hours == 0){
		rain_days[day_counter++] = rain_daily;
		if (day_counter == 9) remove_array_index(rain_days, 0, &day_counter);
		if(sDate.WeekDay == 0){
			rain_weeks[week_counter++] = rain_weekly;
			if (week_counter == 9) remove_array_index(rain_weeks, 0, &week_counter);
		}
		if(sDate.Date == 0){
			rain_months[month_counter++] = rain_monthly;
			if (month_counter == 9)remove_array_index(rain_months, 0, &month_counter);
		}
	}
}

/**************Pluviométrie******************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_ResumeTick();
	if (GPIO_Pin == RAIN_Pin) Flag_EXTI15_RAIN = 1;
	if (GPIO_Pin == TOUCH_Pin) Flag_EXTI15_TOUCH = 1;
}

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
HAL_ResumeTick();
if (htim == &htim7) {
	Flag_TIM7 = 1;
}
}*/
/**************Pluviométrie******************/

/**************Vitesse Du Vent***************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
i++;
if (FIRST_IMP) {
	ccr0 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	FIRST_IMP = 0;
} else {
	ccr1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	TIM1_IC_IT_Flag = 1;
}

}
// calcul de la force du vent selon l'échelle de Beaufort
void WSpeed_To_WForce(float Wind_Speed, uint8_t *Force) {
//passage par paramètre
switch ((int) Wind_Speed) { //Case: [xmin ... xmax[
case 0:
	*Force = 0; //Air calme
	break;
case 1 ... 3:
	*Force = 1; //Air léger
	break;
case 4 ... 7:
	*Force = 2; // Légère brise
	break;
case 8 ... 12:
	*Force = 3; // Brise légère
	break;
case 13 ... 17:
	*Force = 4; // Vent modéré
	break;
case 18 ... 24:
	*Force = 5; // La brise fraîche
	break;
case 25 ... 30:
	*Force = 6; //Forte brise
	break;
case 31 ... 38:
	*Force = 7; //Vent fort
	break;
case 39 ... 46:
	*Force = 8; //Coup de vent
	break;
case 47 ... 54:
	*Force = 9; //Coup de vent de ficelle
	break;
case 55 ... 63:
	*Force = 10; //Tempête
	break;
case 64 ... 73:
	*Force = 11; //Tempête violente
	break;
case 74 ... 1000:
	*Force = 12; //Ouragan
	break;
}
}
/**************Vitesse Du Vent***************/

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
while (1) {
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
