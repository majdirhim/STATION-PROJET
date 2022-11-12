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
#include "tim.h"
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
#define NO_WIND 0.3
#define MPH_CONST 1.492
#define KMH_CONST 1.609 // 1 MPH = 1.609 KM/h
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int _write(int file ,char*ptr,int len){
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
	return len;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void WSpeed_To_WForce(float Wind_Speed,uint8_t* Force);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t TIM1_IC_IT_Flag=0 , FIRST_IMP=1;
volatile uint32_t ccr0 =0 , ccr1=0 ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t First_Speed = 1, Force = 0;
	float Wind_Speed = 0.0, Wind_Speed_KMH = 0.0, Max_Wind = 0.0,
			Min_Wind = 0.0, Frequency = 0.0;
	float Tim1_Freq;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Tim1_Freq=HAL_RCC_GetPCLK2Freq()*2/TIM1->PSC; //APB2_PSC=2 et TIM_psc=5000-1
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(TIM1_IC_IT_Flag){
		  // Calcul de la fréquence dans les deux cas => Avant timer overflow : juste après timer le overflow
		  Frequency = ccr1>=ccr0?(float)Tim1_Freq/(ccr1-ccr0) : (float)Tim1_Freq/((TIM1->ARR+ccr1)-ccr0);
		  // La vitesse du vent(en Mph) correspond à la fréqunce du signal capturée multipliée par une constante
		  Wind_Speed=MPH_CONST*Frequency;
		  if(First_Speed){
			  //Initialiser les Valeur Max et Min de la vitesse du vent(Mph)
			  Min_Wind=Wind_Speed; Max_Wind=Wind_Speed; First_Speed=0;
		  }
		  //CCR1 devient CCR0 pour la prochaine détection d'impulsion
		  ccr0=ccr1;
		  //Si la vitesse est négligeable Wind_Speed = 0
		  Wind_Speed =Wind_Speed>NO_WIND?Wind_Speed:0.0;
		  //Convertir la vitesse en Km/h
		  Wind_Speed_KMH = KMH_CONST*Wind_Speed;
		  // calcul de la maximum et la minimum de la vitesse du vent
		  if(Wind_Speed>Max_Wind)
			  Max_Wind=Wind_Speed;
		  else if (Wind_Speed<Min_Wind && Wind_Speed!=0)
			  Min_Wind=Wind_Speed;
		  //Force du vent selon l'échelle de Beaufort(à interpréter par une disignation dans l'affichage)
		  WSpeed_To_WForce(Wind_Speed,&Force);
		  //Envoie à travers le Port Série la vitesse du vent actuelle
		  printf("Wind_Speed = %.3f Mph %.3f km/h Min=%.3f Max=%.3f Force =%u\n\r",Wind_Speed,Wind_Speed_KMH,Min_Wind,Max_Wind,Force);
		  //Remettre à nouveau le Flag
		  TIM1_IC_IT_Flag=0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(FIRST_IMP){
		ccr0=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		FIRST_IMP=0;
	}
	else{
		ccr1=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		TIM1_IC_IT_Flag=1;
	}

}
//passage par paramètre
void WSpeed_To_WForce(float Wind_Speed,uint8_t* Force){

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
