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
#include "adc.h"
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
#include "stdlib.h"
#include "Carte_Sd.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_lcd.h"
#include "lps22hh_reg.h"
#include "hts221_reg.h"
#include "FontRoboto12.h"
#include "FontRoboto16.h"
#include "FontRoboto20.h"
#include "FontRobotoMedium20.h"
#include "FontRoboto24.h"
#include "FontRoboto32.h"
#include "home_icon_line.h"
#include "sun.h"
#include "settings.h"
#include "settings_home.h"
#include "home.h"
#include "icon_temp.h"
#include "icon_hum.h"
#include "icon_rain.h"
#include "icon_wind.h"
#include "icon_pressure.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/************** WIND SPEED ***************/
#define NO_WIND 0.5 //en KMH
#define MPH_CONST 1.492
#define KMH_CONST 1.609 // 1 MPH = 1.609 KM/h
#define VCC 3.3
#define PULL_RES 6800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/************** FLAGS ***************/
uint8_t Flag_TIM6;
uint8_t Flag_TIM7;
uint8_t Flag_EXTI15_RAIN;
uint8_t Flag_EXTI15_TOUCH;
uint8_t Flag_EXTI11_BTN;
volatile uint8_t Flag_RTCIAA;

/************** RTC ***************/

RTC_DateTypeDef sDate;
RTC_TimeTypeDef sTime;
uint8_t date_buffer[16];
uint8_t date_full_buffer[24];
uint8_t time_buffer[16];
char *Wind_time_buffer[16];
char month_str[4];

/************** LCD TOUCH / DISPLAY ***************/

enum LED {
	RED, GREEN, BLUE
};
enum screens {
	HOME, TH, WIND, RAIN, PRESSURE, SETTINGS
};
TS_StateTypeDef TS_State;
LTDC_HandleTypeDef LtdcHandle;
int touch_x;
int touch_y;
uint8_t touched = 0;
enum screens screen_index = HOME;
uint8_t is_screen_init = 0;
uint8_t user_action = 0;	//Used to measure user inactivity

/************** SD ***************/
SD_State sd_state;
uint8_t sd_state_buffer[32];
/************** RAINFALL ***************/

enum periods {
	HOUR, DAY, WEEK, MONTH
};
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
float rain_hours[10] = { 1, 2, 3, 2, 0, 4, 5, 2, 5, 0 };
float rain_days[10] = { 4, 8, 6, 0, 0, 2, 3, 1, 5, 0 };
float rain_weeks[10] = { 1, 2, 1, 2, 1, 0, 20, 1, 4, 0 };
float rain_months[10] = { 0, 4, 2, 1, 0, 1, 5, 0, 3, 2 };
float pressure_hours[10] = { 1001, 1003, 1005, 990, 985, 990, 995, 997, 1002, 1007 };
uint8_t rain_periods_toggle = 1;

uint16_t hour_counter = 0;
uint16_t day_counter = 0;
uint16_t week_counter = 0;
uint16_t month_counter = 0;
uint16_t rain_events_size = 0;

uint8_t rain_hourly_buffer[16];
uint8_t rain_daily_buffer[16];
uint8_t rain_weekly_buffer[16];
uint8_t rain_monthly_buffer[16];
uint8_t max_period_buffer[16];
uint8_t half_period_buffer[16];

/************** WIND SPEED ***************/
volatile uint8_t TIM1_IC_IT_Flag = 0, FIRST_IMP = 1, i = 0;
volatile uint32_t ccr0 = 0, ccr1 = 0;
uint8_t wind_speed_average_buffer[16];
uint8_t wind_speed_min_buffer[16];
uint8_t wind_speed_max_buffer[16];
char wind_speed_beaufort_buffer[64];

/************** WIND DIR *************/
enum wind_dir {
	Nord = 0, Sud = 180, West = 270, East = 90
};
volatile uint32_t Wind_Dir_Voltage = 0;
volatile uint8_t Wind_Dir_Flag = 0;
uint8_t wind_dir_angle_buffer[64];
char dir_str[16];
/************** WIND DIR *************/
HAL_StatusTypeDef hts221_status;
HAL_StatusTypeDef lps22hh_status;
uint8_t first_hum = 1, first_temp = 1, first_press = 1;
uint8_t temp_buffer[16], temp_min_buffer[16], temp_max_buffer[16];
uint8_t hum_buffer[16], hum_min_buffer[16], hum_max_buffer[16];
uint8_t press_buffer[16], press_min_buffer[16], press_max_buffer[16];
uint8_t init_temp_min_max = 0, init_hum_min_max = 0, init_press_min_max = 0;
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static float pressure_hPa, pressure_min_hPa, pressure_max_hPa;
static float temperature_degC, temperature_degCMax = 0, temperature_degCMin = 100;
static int16_t data_raw_humidity;
static float hum_perc, hum_min_perc, hum_max_perc;
static int16_t data_raw_humidity;
static uint8_t tx_bufferH[1000];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000], tx_bufferpress[1000];
stmdev_ctx_t dev_ctx, dev_ctxHum;
lps22hh_reg_t reg;

const uint16_t hist_x = 74;
const uint16_t hist_y = 220;
const uint16_t hist_w = 200;
const uint16_t hist_h = 110;

const uint16_t graph_x = 220;
const uint16_t graph_y = 220;
const uint16_t graph_w = 200;
const uint16_t graph_h = 110;

const uint16_t circle_radius = 153 / 2;
const uint16_t circle_x = 261 + circle_radius;
const uint16_t circle_y = 61 + circle_radius;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void render_screen(enum screens screen);
void init_screen(enum screens screen);
char* month_string(uint8_t month);

/************** WIND SPEED ***************/
void WSpeed_To_WForce(float Wind_Speed, uint8_t *Force);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

/************** WIND DIR *************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_writeH(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_readH(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
void gettemperature();
void getpression();
void gethumidity();
void set_RGB_LED(enum LED, uint8_t on);
float min_array_value(float *arr, uint16_t len);
float max_array_value(float *arr, uint16_t len);
void normalize_array(float *arr, uint16_t len, float min_data, float max_norm, float max_data, float *res);
void render_hist(float *periods, uint16_t len);
void render_graph(float *periods, uint16_t len);
pPoint triangle(int8_t base, int8_t height, uint16_t x, uint16_t y);
void update_wind_dir(float angle);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
	float x0;
	float y0;
	float x1;
	float y1;
} lin_t;

float linear_interpolation(lin_t *lin, int16_t x) {
	return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) - (lin->x0 * lin->y1))) / (lin->x1 - lin->x0);
}

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

extern void RTC_SetDate(RTC_DateTypeDef *sDate, uint8_t year, uint8_t month, uint8_t date, uint8_t wday);
extern void RTC_SetTime(RTC_TimeTypeDef *sTime, uint8_t hour, uint8_t min, uint8_t sec);

void remove_rain_event(unsigned int idx) {
	for (uint16_t i = idx; i < rain_events_size; i++) {
		rain_events[i] = rain_events[i + 1];
	}
	rain_events_size--;
}

void remove_array_index(float *arr, uint16_t idx, uint16_t *elem_count) {
	for (unsigned int i = idx; i < 10; i++) {
		arr[i] = arr[i + 1];
	}
	(*elem_count)--;
	printf("Remove index %d, resulting count : %d\n\r", idx, *elem_count);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/************** WIND SPEED ***************/
	uint8_t First_Speed = 1, Force = 0, LastForce = 0;
	uint16_t W_nb = 0, Average_sum = 0;
	float Wind_Speed = 0.0, Wind_Speed_KMH = 0.0, Max_Wind = 0.0, Min_Wind = 0.0, Frequency = 0.0, Speed_Sum = 0.0, Average_Speed_Sum = 0.0, Average_Wind_Speed_KMH = 0.0, Hour_Wind_Average = 0.0;
	float Tim1_Freq;
	/************** WIND SPEED ***************/
	/************** WIND DIR *************/
	float UR = 0.0;
	double Res = 0.0;
	float dir = 0.0;
	/************** WIND DIR *************/
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;

	dev_ctxHum.write_reg = platform_writeH;
	dev_ctxHum.read_reg = platform_readH;
	dev_ctxHum.handle = &hi2c1;
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
	MX_ADC1_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	RTC_SetDate(&sDate, 22, 11, 9, 2);
	RTC_SetTime(&sTime, 11, 00, 00);
	sprintf((char*) date_buffer, "%02d %s", sDate.Date, month_string(sDate.Month));
	sprintf((char*) date_full_buffer, "%02d %s 20%d", sDate.Date, month_string(sDate.Month), sDate.Year);
	sprintf((char*) time_buffer, "%02d:%02d", sTime.Hours, sTime.Minutes);

user_action = 1;
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	sprintf((char*) rain_hourly_buffer, "%6.1f", rain_hourly);
	sprintf((char*) rain_daily_buffer, "%6.1f", rain_daily);
	sprintf((char*) rain_weekly_buffer, "%6.1f", rain_weekly);
	sprintf((char*) rain_monthly_buffer, "%6.1f", rain_monthly);

	sprintf((char*) wind_speed_average_buffer, "%6.1f", 0.);
	sprintf((char*) wind_speed_min_buffer, "%6.1f", 0.);
	sprintf((char*) wind_speed_max_buffer, "%6.1f", 0.);
	sprintf((char*) wind_speed_beaufort_buffer, "%s", "Air calme");
	sprintf((char*) wind_dir_angle_buffer, "%5.1f", 0.);
	strcpy(dir_str, "West");

	whoamI = 0;
	lps22hh_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != LPS22HH_ID) while (1)
		; /*manage here device not found */
	/* Restore default configuration */
	lps22hh_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lps22hh_reset_get(&dev_ctx, &rst);
	}
	while (rst);

	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lps22hh_data_rate_set(&dev_ctx, LPS22HH_10_Hz_LOW_NOISE);

	/**************Vitesse Du Vent***************/
	Tim1_Freq = HAL_RCC_GetPCLK2Freq() / TIM1->PSC; //APB2_PSC=1 et TIM_psc=5000-1
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

	/**************SD Card***********************/
	Fat_Init();
	WR_TO_Sd("Wind.csv","Timestamp,Average_Wind_Speed,Min,Max,Force\n"); //Init les colonnes dans le fichier CSV
	WR_TO_Sd("rain.csv","rain_mm_hours\n"); //Init les colonnes dans le fichier CSV
	sd_state = Sd_Space();
	sprintf((char*) sd_state_buffer, "%0.2f/%0.2f Gb", sd_state.Total_Space - sd_state.Free_Space, sd_state.Total_Space);
	/**************SD Card***********************/
	/**************LCD TOUCH / DISPLAY***************/
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, SDRAM_DEVICE_ADDR);
	BSP_LCD_SetLayerVisible(LTDC_ACTIVE_LAYER, ENABLE);
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_TS_Init(480, 272);
	BSP_TS_ITConfig();
	render_screen(screen_index);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (Flag_RTCIAA == 1) {
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			printf("%s : %d/%d [%dh]\n\r", "Hourly alarm", sDate.Date, sDate.Month, sTime.Hours);
			sprintf((char*) date_buffer, "%02d %s", sDate.Date, month_string(sDate.Month));
			sprintf((char*) date_full_buffer, "%02d %s 20%d", sDate.Date, month_string(sDate.Month), sDate.Year);
			rain_hours[hour_counter++] = rain_hourly;
			if (hour_counter == 9) remove_array_index(rain_hours, 0, &hour_counter);
			if (sTime.Hours == 0) {
				rain_days[day_counter++] = rain_daily;
				if (day_counter == 9) remove_array_index(rain_days, 0, &day_counter);
				if (sDate.WeekDay == 0) {
					rain_weeks[week_counter++] = rain_weekly;
					if (week_counter == 9) remove_array_index(rain_weeks, 0, &week_counter);
				}
				if (sDate.Date == 0) {
					rain_months[month_counter++] = rain_monthly;
					if (month_counter == 9) remove_array_index(rain_months, 0, &month_counter);
				}
			}
			timestamp = epoch_days_fast(sDate.Year + 2000, sDate.Month, sDate.Date) * DAY_SECONDS + (sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds);
			WR_TO_Sd("rain.csv", "%d, %.2fmm", timestamp, rain_hourly); 	//Ecriture dans le fichier rain.csv
			/**********Wind Speed********/
			Hour_Wind_Average = (float) Average_Speed_Sum / Average_sum;
			Average_Speed_Sum = 0.0;
			Average_sum = 0;
			WR_TO_Sd("Wind.csv", "%02d/%02d/%02d %02d:%02d:%02d,%.3f, %.3f, %.3f,%u", 2000 + sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds, Hour_Wind_Average, Min_Wind,
					Max_Wind, Force); //ecriture dans le fichier wind.txt
			Hour_Wind_Average = 0.0;

			sd_state = Sd_Space();
			sprintf((char*) sd_state_buffer, "%0.2f/%0.2f Gb", sd_state.Total_Space - sd_state.Free_Space, sd_state.Total_Space);
			Flag_RTCIAA = 0;
		}
		/**************RAINFALL******************/
		if (Flag_EXTI15_RAIN == 1) {
			/* Get the RTC current Date */
			HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			timestamp = epoch_days_fast(sDate.Year + 2000, sDate.Month, sDate.Date) * DAY_SECONDS + (sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds);
			printf("Date : %02u:%02u:%04u ", sDate.Date, sDate.Month, 2000 + sDate.Year);
			printf("@ %02u:%02u:%02u\n\r", sTime.Hours, sTime.Minutes, sTime.Seconds);
			printf("Timestamp : %lu\n\r", timestamp);

			printf("%d Rain events.\n\r", rain_events_size + 1);
			rain_events[rain_events_size] = timestamp;
			rain_events_size++;
			rain_hourly = 0;
			rain_daily = 0;
			rain_weekly = 0;
			rain_monthly = 0;
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
				}
				else
				//remove_array_index(rain_events, i, &rain_events_size);
				remove_rain_event(i);
			}
			sprintf((char*) rain_hourly_buffer, "%6.1f", rain_hourly);
			sprintf((char*) rain_daily_buffer, "%6.1f", rain_daily);
			sprintf((char*) rain_weekly_buffer, "%6.1f", rain_weekly);
			sprintf((char*) rain_monthly_buffer, "%6.1f", rain_monthly);
			printf("Rain h %.2fmm, %.2fmm, w %.2fmm, m %.2fmm \n\r", rain_hourly, rain_daily, rain_weekly, rain_monthly);
			printf("--------------------------------\n\r");
			Flag_EXTI15_RAIN = 0;
		}
		if (Flag_TIM7 == 1) {
			printf("TIM7 Flag callback %d.\n\r", user_action);
			if(user_action==0){
				BSP_LCD_DisplayOff();

			}
			else {
				user_action = 0;
				//htim7.Instance->EGR = 1;
				//TIM7->ARR += TIM7->CNT;
			}
			Flag_TIM7 = 0;
		}
		/*else {
		 HAL_SuspendTick();
		 HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		 }*/

		if (Flag_EXTI11_BTN == 1) {
			GPIO_PinState screen_state = HAL_GPIO_ReadPin(LCD_DISP_GPIO_PORT, LCD_DISP_PIN);
			if (screen_state)
				BSP_LCD_DisplayOff();
			else BSP_LCD_DisplayOn();
			Flag_EXTI11_BTN = 0;
		}
		/**************LCD TOUCH***************/
		if (Flag_EXTI15_TOUCH == 1) {
			BSP_TS_GetState(&TS_State);

			if (TS_State.touchDetected) {
				/* Get X and Y position of the touch post calibrated */
				touch_x = TS_State.touchX[0];
				touch_y = TS_State.touchY[0];
				touched = 1;
				//if (TS_State.touchEventId[0] == 2 || TS_State.touchEventId[0] == 1) {
				printf("Touched lcd (x : %d, y : %d), %d", touch_x, touch_y, TS_State.touchEventId[0]);
				switch (screen_index) {
				case HOME:
					printf("@HOME screen\n\r");
					if (touch_x > 60 && touch_x < 130 && touch_y > 150 && touch_y < 220) {
						screen_index = TH;
						is_screen_init = 0;
						printf("> TH button\n\r");
					}
					else if (touch_x > 160 && touch_x < 210 && touch_y > 150 && touch_y < 220) {
						screen_index = WIND;
						is_screen_init = 0;
						printf("> WIND button\n\r");
					}
					else if (touch_x > 270 && touch_x < 310 && touch_y > 150 && touch_y < 220) {
						screen_index = RAIN;
						is_screen_init = 0;
						printf("> RAIN button\n\r");
					}
					else if (touch_x > 350 && touch_x < 430 && touch_y > 150 && touch_y < 220) {
						screen_index = PRESSURE;
						is_screen_init = 0;
						printf("> PRESSURE button\n\r");
					}
					else if (touch_x > 440 && touch_x < 480 && touch_y > 0 && touch_y < 40) {
						screen_index = SETTINGS;
						is_screen_init = 0;
						printf("> SETTINGS button\n\r");
					}
					break;
				case TH:
					printf("@TH screen\n\r");
					if (touch_x > 0 && touch_x < 40 && touch_y > 0 && touch_y < 40) {
						screen_index = HOME;
						is_screen_init = 0;
						printf("> HOME button\n\r");
					}
					else if (touch_x > 440 && touch_x < 480 && touch_y > 0 && touch_y < 40) {
						screen_index = SETTINGS;
						is_screen_init = 0;
						printf("> SETTINGS button\n\r");
					}
					break;
				case WIND:
					printf("@WIND screen\n\r");
					if (touch_x > 0 && touch_x < 40 && touch_y > 0 && touch_y < 40) {
						screen_index = HOME;
						is_screen_init = 0;
						printf("> HOME button\n\r");
					}
					else if (touch_x > 440 && touch_x < 480 && touch_y > 0 && touch_y < 40) {
						screen_index = SETTINGS;
						is_screen_init = 0;
						printf("> SETTINGS button\n\r");
					}
					break;
				case RAIN:
					printf("@RAIN screen\n\r");
					if (touch_x > 0 && touch_x < 40 && touch_y > 0 && touch_y < 40) {
						screen_index = HOME;
						is_screen_init = 0;
						printf("> HOME button\n\r");
					}
					else if (touch_x > 440 && touch_x < 480 && touch_y > 0 && touch_y < 40) {
						screen_index = SETTINGS;
						is_screen_init = 0;
						printf("> SETTINGS button\n\r");
					}
					else if (touch_x > 340 && touch_x < 440 && touch_y > 20 && touch_y < 60) {
						rain_periods_toggle = HOUR;
						is_screen_init = 0;
						printf("> Toggle rain per hours\n\r");
					}
					else if (touch_x > 340 && touch_x < 440 && touch_y > 80 && touch_y < 110) {
						rain_periods_toggle = DAY;
						is_screen_init = 0;
						printf("> Toggle rain per days\n\r");
					}
					else if (touch_x > 340 && touch_x < 440 && touch_y > 130 && touch_y < 160) {
						rain_periods_toggle = WEEK;
						is_screen_init = 0;
						printf("> Toggle rain per weeks\n\r");
					}
					else if (touch_x > 340 && touch_x < 440 && touch_y > 190 && touch_y < 220) {
						rain_periods_toggle = MONTH;
						is_screen_init = 0;
						printf("> Toggle rain per months\n\r");
					}

					break;
				case PRESSURE:
					printf("@PRESSURE screen\n\r");
					if (touch_x > 0 && touch_x < 40 && touch_y > 0 && touch_y < 40) {
						screen_index = HOME;
						printf("> HOME button\n\r");
					}
					else if (touch_x > 440 && touch_x < 480 && touch_y > 0 && touch_y < 40) {
						screen_index = SETTINGS;
						is_screen_init = 0;
						printf("> SETTINGS button\n\r");
					}
					is_screen_init = 0;
					break;
				case SETTINGS:
					printf("@SETTINGS screen\n\r");
					if (touch_x > 0 && touch_x < 40 && touch_y > 0 && touch_y < 40) {
						screen_index = HOME;
						printf("> HOME button\n\r");
					}
					else if (touch_x > 40 && touch_x < 70 && touch_y > 60 && touch_y < 90) {
						printf("> Date++ \n\r");
						RTC_SetDate(&sDate, sDate.Year, sDate.Month, sDate.Date + 1, 0);
						HAL_Delay(50);
					}
					else if (touch_x > 40 && touch_x < 70 && touch_y > 110 && touch_y < 140) {
						printf("> Date-- \n\r");
						RTC_SetDate(&sDate, sDate.Year, sDate.Month, sDate.Date - 1, 0);
						HAL_Delay(50);
					}
					else if (touch_x > 100 && touch_x < 130 && touch_y > 60 && touch_y < 90) {
						printf("> Month++ \n\r");
						RTC_SetDate(&sDate, sDate.Year, sDate.Month + 1, sDate.Date, 0);
						HAL_Delay(50);
					}
					else if (touch_x > 100 && touch_x < 130 && touch_y > 110 && touch_y < 140) {
						printf("> Month-- \n\r");
						RTC_SetDate(&sDate, sDate.Year, sDate.Month - 1, sDate.Date, 0);
						HAL_Delay(50);
					}
					else if (touch_x > 175 && touch_x < 205 && touch_y > 60 && touch_y < 90) {
						printf("> Year++ \n\r");
						RTC_SetDate(&sDate, sDate.Year + 1, sDate.Month, sDate.Date, 0);
						HAL_Delay(50);
					}
					else if (touch_x > 175 && touch_x < 205 && touch_y > 110 && touch_y < 140) {
						printf("> Year-- \n\r");
						RTC_SetDate(&sDate, sDate.Year - 1, sDate.Month, sDate.Date, 0);
						HAL_Delay(50);
					}
					else if (touch_x > 90 && touch_x < 120 && touch_y > 150 && touch_y < 180) {
						printf("> Hour++ \n\r");
						RTC_SetTime(&sTime, sTime.Hours + 1, sTime.Minutes, sTime.Seconds);
						HAL_Delay(50);
					}
					else if (touch_x > 90 && touch_x < 120 && touch_y > 215 && touch_y < 245) {
						printf("> Hour-- \n\r");
						RTC_SetTime(&sTime, sTime.Hours - 1, sTime.Minutes, sTime.Seconds);
						HAL_Delay(50);
					}
					else if (touch_x > 140 && touch_x < 170 && touch_y > 150 && touch_y < 180) {
						printf("> Minute++ \n\r");
						RTC_SetTime(&sTime, sTime.Hours, sTime.Minutes + 1, sTime.Seconds);
						HAL_Delay(50);
					}
					else if (touch_x > 140 && touch_x < 170 && touch_y > 215 && touch_y < 245) {
						printf("> Minute-- \n\r");
						RTC_SetTime(&sTime, sTime.Hours, sTime.Minutes - 1, sTime.Seconds);
						HAL_Delay(50);
					}
					sprintf((char*) time_buffer, "%02d:%02d", sTime.Hours, sTime.Minutes);
					sprintf((char*) date_buffer, "%02d %s", sDate.Date, month_string(sDate.Month));
					sprintf((char*) date_full_buffer, "%02d %s 20%d", sDate.Date, month_string(sDate.Month), sDate.Year);

					is_screen_init = 0;
					break;
					//}

				}
			}
			user_action = 1;

			TIM7->EGR = 1;
			render_screen(screen_index);
			BSP_TS_ResetTouchData(&TS_State);
			BSP_TS_ITClear();
			Flag_EXTI15_TOUCH = 0;
		}
		if (Flag_TIM6 == 1) {
			printf("TIM6 Flag callback.\n\r");
			if (screen_index != SETTINGS) {
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				sprintf((char*) date_buffer, "%02d %s", sDate.Date, month_string(sDate.Month));
				sprintf((char*) date_full_buffer, "%02d %s 20%d", sDate.Date, month_string(sDate.Month), sDate.Year);
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				sprintf((char*) time_buffer, "%02d:%02d", sTime.Hours, sTime.Minutes);
			}

			gettemperature();
			getpression();
			gethumidity();
			render_screen(screen_index);

			Flag_TIM6 = 0;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/************** WIND SPEED***************/
		if (TIM1_IC_IT_Flag) {
			// Calcul de la fréquence dans les deux cas => Avant timer overflow : juste après timer le overflow
			Frequency = ccr1 >= ccr0 ? (float) Tim1_Freq / (ccr1 - ccr0) : (float) Tim1_Freq / ((TIM1->ARR + ccr1) - ccr0);
			//CCR1 prend la valeur de CCR0 pour la prochaine détection d'impulsion
			ccr0 = ccr1;
			// La vitesse du vent(en Mph) correspond à la fréqunce du signal capturée multipliée par une constante
			Wind_Speed = MPH_CONST * Frequency;
			//Convertir la vitesse en Km/h
			Wind_Speed_KMH = KMH_CONST * Wind_Speed;
			if (First_Speed) {
				//Initialiser les Valeur Max et Min de la vitesse du vent(Mph)
				Min_Wind = Wind_Speed;
				Max_Wind = Wind_Speed;
				First_Speed = 0;
			}
			//Si la vitesse est négligeable Wind_Speed = 0
			if (Wind_Speed_KMH < NO_WIND) {
				Wind_Speed_KMH = 0.0;
				Wind_Speed = 0.0;
			}
			// calcul de le maximum et le minimum de la vitesse du vent
			if (Wind_Speed > Max_Wind)
				Max_Wind = Wind_Speed;
			else if (Wind_Speed < Min_Wind && Wind_Speed != 0) Min_Wind = Wind_Speed;
			// calculer la somme des vitesses ( à diviser après par W_nb pour déterminer la moyenne )
			Speed_Sum += Wind_Speed_KMH;
			++W_nb; //nombre total
			//Force du vent selon l'échelle de Beaufort(à interpréter par une disignation dans l'affichage)
			WSpeed_To_WForce(Wind_Speed, &Force);
			//Envoie à travers le Port Série la vitesse du vent actuelle
			printf("Wind_Speed = %.3f km/h Min=%.3f Max=%.3f Force =%u\n\r", Wind_Speed_KMH, Min_Wind, Max_Wind, Force);
			//Calcul de la moyenne à chaque changement de Force de Beaufort supérieur à 2 (Légère brise)
			if (LastForce != Force && Force >= 2) {
				Average_Wind_Speed_KMH = (float) Speed_Sum / W_nb;
				Average_Speed_Sum += Average_Wind_Speed_KMH;
				++Average_sum; //Pour la moyenne d'une heure
				Average_Wind_Speed_KMH = 0.0;
				W_nb = 0;
				Speed_Sum = 0;
				LastForce = Force;
			}
			/******Affichage*****/
			sprintf((char*) wind_speed_average_buffer, "%5.1f", Average_Wind_Speed_KMH);
			sprintf((char*) wind_speed_min_buffer, "%5.1f", Min_Wind);
			sprintf((char*) wind_speed_max_buffer, "%5.1f", Max_Wind);
			/******Affichage*****/
			//Déclencher la mesure de direction
			HAL_ADC_Start_IT(&hadc1);
			//Remettre à nouveau le Flag
			TIM1_IC_IT_Flag = 0;
		}

		/************** Direction Du vent ************/
		if (Wind_Dir_Flag) {
			UR = (float) (Wind_Dir_Voltage * 3.3 / 4095); //Calculer la tension en Volts
			Res = UR * PULL_RES / (VCC - UR); // calculer la résistance
			switch ((unsigned int) Res) {
			case 33000 ... 36000:
				dir = Nord;
				strcpy(dir_str, "North");
				break;
			case 6570 ... 6600:
				dir = 22.5;
				break;
			case 8000 ... 9800:
				dir = 45;
				strcpy(dir_str, "North-East");
				break;
			case 890000 ... 900000:
				dir = 67.5;
				break;
			case 900 ... 1200:
				dir = East;
				strcpy(dir_str, "East");
				break;
			case 690 ... 800:
				dir = 112.5;
			case 2000 ... 2500:
				dir = 135;
				strcpy(dir_str, "South-East");
				break;
			case 1300 ... 1500:
				dir = 157.5;
				break;
			case 3800 ... 5000:
				dir = Sud;
				strcpy(dir_str, "South");
				break;
			case 3000 ... 3300:
				dir = 202.5;
				break;
			case 15800 ... 16300:
				dir = 225;
				strcpy(dir_str, "South-West");
				break;
			case 14000 ... 14300:
				dir = 247.5;
				break;
			case 116000 ... 155000:
				dir = West;
				strcpy(dir_str, "West");
				break;
			case 40000 ... 43000:
				dir = 292.5;
				break;
			case 64000 ... 65100:
				dir = 315;
				break;
			case 20000 ... 23000:
				dir = 337.5;
				break;

			}
			/****Affichage*****/
			update_wind_dir(dir);
			sprintf((char*) wind_dir_angle_buffer, "%5.1f", dir);
			/****Affichage*****/
			printf("Resistance :%lf\n\r", Res); //debug
			printf("%.1f degré\n\r", dir);
			Wind_Dir_Flag = 0;
		}
		/************** Direction Du vent************/
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
char* month_string(uint8_t month) {
	switch (month) {
	case 0:
		strcpy(month_str, "Jan");
		break;
	case 1:
		strcpy(month_str, "Feb");
		break;
	case 2:
		strcpy(month_str, "Mar");
		break;
	case 3:
		strcpy(month_str, "Apr");
		break;
	case 4:
		strcpy(month_str, "May");
		break;
	case 5:
		strcpy(month_str, "Jun");
		break;
	case 6:
		strcpy(month_str, "Jul");
		break;
	case 7:
		strcpy(month_str, "Aug");
		break;
	case 8:
		strcpy(month_str, "Sep");
		break;
	case 9:
		strcpy(month_str, "Oct");
		break;
	case 10:
		strcpy(month_str, "Nov");
		break;
	case 11:
		strcpy(month_str, "Dec");
		break;
	}
	return month_str;
}

void init_screen(enum screens screen) {
	if (screen != HOME) {
		BSP_LCD_Clear((uint32_t) 0xFFF2F2F2);
		BSP_LCD_SetBackColor((uint32_t) 0xFFF2F2F2);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_SetFont(&LCD_FONT_16);
		BSP_LCD_DrawBitmap(10, 10, (uint8_t*) home);
		BSP_LCD_DisplayStringAt(50, 15, (uint8_t*) date_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(150, 15, (uint8_t*) time_buffer, LEFT_MODE);
	}
	switch (screen) {
	case HOME:
//	HAL_LTDC_ConfigColorKeying_NoReload(&LtdcHandle, 0x00000000, LTDC_LAYER_1);
//	HAL_LTDC_EnableColorKeying_NoReload(&LtdcHandle, LTDC_LAYER_1);
		BSP_LCD_Clear((uint32_t) 0xFF649DE1);
		BSP_LCD_DrawBitmap(446, 10, (uint8_t*) settings);
		BSP_LCD_DrawBitmap(0, 0, (uint8_t*) sun);
		BSP_LCD_DrawBitmap(0, 149, (uint8_t*) home_icon_line);
		BSP_LCD_DrawBitmap(446, 10, (uint8_t*) settings_home);
		is_screen_init = 1;
		break;
	case TH:
		BSP_LCD_DrawBitmap(446, 10, (uint8_t*) settings);
		BSP_LCD_DrawBitmap(122, 73, (uint8_t*) icon_temp);
		BSP_LCD_DrawBitmap(341, 76, (uint8_t*) icon_hum);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_DrawLine(130, 173, 130, 173 + 29);
		BSP_LCD_DrawLine(350, 173, 350, 173 + 29);
		is_screen_init = 1;
		break;
	case WIND:
		BSP_LCD_DrawBitmap(446, 10, (uint8_t*) settings);
		BSP_LCD_DrawBitmap(85, 71, (uint8_t*) icon_wind);
		BSP_LCD_DisplayStringAt(252 + 73, 29, (uint8_t*) "N", LEFT_MODE);
		BSP_LCD_DisplayStringAt(271 + 146, 130, (uint8_t*) "E", LEFT_MODE);
		BSP_LCD_DisplayStringAt(252 + 73, 215, (uint8_t*) "S", LEFT_MODE);
		BSP_LCD_DisplayStringAt(231, 130, (uint8_t*) "O", LEFT_MODE);
		BSP_LCD_DrawLine(110, 173, 110, 173 + 29);
		BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
		BSP_LCD_DrawCircle(261 + 73, 61 + 73, 73);
		BSP_LCD_DrawCircle(261 + 73, 61 + 73, 74);
		BSP_LCD_DrawCircle(261 + 73, 61 + 73, 75);
		is_screen_init = 1;
		break;
	case RAIN:
		BSP_LCD_DrawBitmap(446, 10, (uint8_t*) settings);
		BSP_LCD_DrawBitmap(144, 28, (uint8_t*) icon_rain);
		BSP_LCD_SetTextColor((uint32_t) 0xFFBABABA);
		BSP_LCD_FillCircle(354, 42, 10);
		BSP_LCD_FillCircle(354, 95, 10);
		BSP_LCD_FillCircle(354, 149, 10);
		BSP_LCD_FillCircle(354, 204, 10);
		BSP_LCD_SetFont(&LCD_FONT_16);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_DisplayStringAt(371, 37, (uint8_t*) "hour", LEFT_MODE);
		BSP_LCD_DisplayStringAt(371, 90, (uint8_t*) "day", LEFT_MODE);
		BSP_LCD_DisplayStringAt(371, 144, (uint8_t*) "week", LEFT_MODE);
		BSP_LCD_DisplayStringAt(371, 199, (uint8_t*) "month", LEFT_MODE);
		is_screen_init = 1;
		break;
	case PRESSURE:
		BSP_LCD_DrawBitmap(446, 10, (uint8_t*) settings);
		BSP_LCD_DrawBitmap(85, 48, (uint8_t*) icon_pressure);
		BSP_LCD_SetBackColor((uint32_t) 0xFFF2F2F2);
		is_screen_init = 1;
		break;
	case SETTINGS:
		BSP_LCD_SetBackColor((uint32_t) 0xFFF2F2F2);
//		Point pt1 = { .X = 0, .Y= 0 };
//		Point pt2 = { .X = 20, .Y= 0 };
//		Point pt3 = { .X = 10, .Y= 15 };
//		Point  tri[3] = {pt1,pt2,pt3};
//		BSP_LCD_FillPolygon(&tri,3);
//		Point pt1_r = { .X = 40, .Y= 15 };
//		Point pt2_r = { .X = 60, .Y= 15 };
//		Point pt3_r = { .X = 50, .Y= 0 };
//		Point  tri_r[3] = {pt1_r,pt2_r,pt3_r};
		BSP_LCD_FillPolygon(triangle(20, -15, 105, 80), 3);
		BSP_LCD_FillPolygon(triangle(20, -15, 45, 80), 3);
		BSP_LCD_FillPolygon(triangle(20, -15, 185, 80), 3);
		BSP_LCD_FillPolygon(triangle(20, 15, 105, 120), 3);
		BSP_LCD_FillPolygon(triangle(20, 15, 45, 120), 3);
		BSP_LCD_FillPolygon(triangle(20, 15, 185, 120), 3);

		BSP_LCD_FillPolygon(triangle(20, -15, 93, 190), 3);
		BSP_LCD_FillPolygon(triangle(20, -15, 145, 190), 3);
		BSP_LCD_FillPolygon(triangle(20, 15, 93, 230), 3);
		BSP_LCD_FillPolygon(triangle(20, 15, 145, 230), 3);

		BSP_LCD_DisplayStringAt(282, 90, (uint8_t*) "Storage", LEFT_MODE);
		is_screen_init = 1;
		break;
	}
}

pPoint triangle(int8_t base, int8_t height, uint16_t x, uint16_t y) {
	Point pt1 = { .X = x, .Y = y };
	Point pt2 = { .X = x + base, .Y = y };
	Point pt3 = { .X = x + (base / 2), .Y = y + height };
	Point *tri = NULL;
	tri = (Point*) malloc(sizeof(Point) * 3);
	tri[0] = pt1;
	tri[1] = pt2;
	tri[2] = pt3;
	//Point tri[3] = {pt1,pt2,pt3};
	return (pPoint) tri;
}

void render_screen(enum screens screen) {
	if (!is_screen_init) init_screen(screen);

	switch (screen) {
	case HOME:
		BSP_LCD_SetFont(&LCD_FONT_24);
		BSP_LCD_SetBackColor((uint32_t) 0xFF649DE1);
		BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
		BSP_LCD_DisplayStringAt(0, 20, (uint8_t*) date_buffer, CENTER_MODE);
		BSP_LCD_SetFont(&LCD_FONT_MEDIUM_20);
		BSP_LCD_DisplayStringAt(0, 60, (uint8_t*) time_buffer, CENTER_MODE);
		BSP_LCD_SetFont(&LCD_FONT_16);
		BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
		BSP_LCD_DisplayStringAt(60, 200, (uint8_t*) temp_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(135, 200, (uint8_t*) wind_speed_average_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(235, 200, (uint8_t*) rain_daily_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(350, 200, (uint8_t*) press_buffer, LEFT_MODE);
		break;
	case TH:
		BSP_LCD_SetFont(&LCD_FONT_20);
		BSP_LCD_SetBackColor((uint32_t) 0xFFF2F2F2);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_DisplayStringAt(90, 140, (uint8_t*) temp_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(300, 140, (uint8_t*) hum_buffer, LEFT_MODE);
		BSP_LCD_SetFont(&LCD_FONT_16);
		BSP_LCD_SetTextColor((uint32_t) 0xFF0B7AE0);
		BSP_LCD_DisplayStringAt(50, 180, (uint8_t*) temp_min_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(260, 180, (uint8_t*) hum_min_buffer, LEFT_MODE);
		BSP_LCD_SetTextColor((uint32_t) 0xFFDF3535);
		BSP_LCD_DisplayStringAt(150, 180, (uint8_t*) temp_max_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(360, 180, (uint8_t*) hum_max_buffer, LEFT_MODE);
		break;
	case WIND:
		BSP_LCD_SetFont(&LCD_FONT_20);
		BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_DisplayStringAt(35, 135, (uint8_t*) wind_speed_average_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(35, 200, (uint8_t*) wind_speed_beaufort_buffer, LEFT_MODE);
		BSP_LCD_SetFont(&LCD_FONT_16);
		BSP_LCD_SetTextColor((uint32_t) 0xFF0B7AE0);
		BSP_LCD_DisplayStringAt(0, 175, (uint8_t*) wind_speed_min_buffer, LEFT_MODE);
		BSP_LCD_SetTextColor((uint32_t) 0xFFDF3535);
		BSP_LCD_DisplayStringAt(100, 175, (uint8_t*) wind_speed_max_buffer, LEFT_MODE);
		break;
	case RAIN:
		BSP_LCD_SetFont(&LCD_FONT_20);
		BSP_LCD_SetBackColor((uint32_t) 0xFFF2F2F2);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_DisplayStringAt(40, 59, (uint8_t*) rain_hourly_buffer, RIGHT_MODE);
		BSP_LCD_DisplayStringAt(40, 112, (uint8_t*) rain_daily_buffer, RIGHT_MODE);
		BSP_LCD_DisplayStringAt(40, 166, (uint8_t*) rain_weekly_buffer, RIGHT_MODE);
		BSP_LCD_DisplayStringAt(40, 221, (uint8_t*) rain_monthly_buffer, RIGHT_MODE);
		BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
		BSP_LCD_FillCircle(354, 42, 6);
		BSP_LCD_FillCircle(354, 95, 6);
		BSP_LCD_FillCircle(354, 149, 6);
		BSP_LCD_FillCircle(354, 204, 6);
		BSP_LCD_SetTextColor((uint32_t) 0xFF649DE1);
		switch (rain_periods_toggle) {
		case HOUR:
			BSP_LCD_FillCircle(354, 42, 6);
			render_hist(rain_hours, 10);
			break;
		case DAY:
			BSP_LCD_FillCircle(354, 95, 6);
			render_hist(rain_days, 10);
			break;
		case WEEK:
			BSP_LCD_FillCircle(354, 149, 6);
			render_hist(rain_weeks, 10);
			break;
		case MONTH:
			BSP_LCD_FillCircle(354, 204, 6);
			render_hist(rain_months, 10);
			break;
		}
		break;
	case PRESSURE:
		BSP_LCD_SetFont(&LCD_FONT_20);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_DisplayStringAt(58, 115, (uint8_t*) press_buffer, LEFT_MODE);
		BSP_LCD_SetFont(&LCD_FONT_16);
		BSP_LCD_SetTextColor((uint32_t) 0xFF0B7AE0);
		BSP_LCD_DisplayStringAt(20, 150, (uint8_t*) press_min_buffer, LEFT_MODE);
		BSP_LCD_SetTextColor((uint32_t) 0xFFDF3535);
		BSP_LCD_DisplayStringAt(120, 150, (uint8_t*) press_max_buffer, LEFT_MODE);
		render_graph(pressure_hours, 10);
		break;
	case SETTINGS:
		BSP_LCD_SetFont(&LCD_FONT_20);
		BSP_LCD_DisplayStringAt(35, 90, (uint8_t*) date_full_buffer, LEFT_MODE);
		BSP_LCD_DisplayStringAt(85, 200, (uint8_t*) time_buffer, LEFT_MODE);
		BSP_LCD_SetFont(&LCD_FONT_12);
		BSP_LCD_DisplayStringAt(282, 120, (uint8_t*) sd_state_buffer, LEFT_MODE);
		break;
	}
}

//Hourly alarm to save data
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	HAL_ResumeTick();
	Flag_RTCIAA = 1;
}

/************** RAINFALL ******************/

float min_array_value(float *arr, uint16_t len) {
	float min = arr[0];
	for (uint16_t i = 0; i < len; i++) {
		printf("%f\n\r", arr[i]);
		min = arr[i] < min ? arr[i] : min;
	}
	return min;
}

float max_array_value(float *arr, uint16_t len) {
	float max = arr[0];
	for (uint16_t i = 0; i < len; i++)
		max = arr[i] > max ? arr[i] : max;
	return max;
}

void normalize_array(float *arr, uint16_t len, float min_data, float max_norm, float max_data, float *res) {
	for (uint16_t i = 0; i < len; i++) {
		res[i] = (arr[i] - min_data) * max_norm / (max_data - min_data);
	}
}

void render_hist(float *periods, uint16_t len) {
// Normalize data to histogram height
	float max_period = max_array_value(periods, 10);
	sprintf((char*) max_period_buffer, "%6.1f", max_period);
	sprintf((char*) half_period_buffer, "%6.1f", max_period / 2);
	float periods_normalized[10];
	normalize_array(periods, len, 0, hist_h, max_period, periods_normalized);
// Draw histogram
	BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
	BSP_LCD_DrawLine(hist_x, hist_y, hist_x + hist_w, hist_y);
	BSP_LCD_DrawDottedLine(hist_x, hist_y - hist_h / 2, hist_x + hist_w, hist_y - hist_h / 2, 8);
	BSP_LCD_DrawDottedLine(hist_x, hist_y - hist_h, hist_x + hist_w, hist_y - hist_h, 8);
	BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
	int bin_w = hist_w / len;
//printf("%s\n\r", "Hist bars height values");
	for (uint16_t i = 0; i < len; i++) {
		BSP_LCD_FillRect(1 + hist_x + i * bin_w, hist_y - periods_normalized[i] - 1, bin_w - 2, periods_normalized[i]);
		//printf("%f\n\r", periods_normalized[i]);
	}
	BSP_LCD_SetFont(&LCD_FONT_12);
	BSP_LCD_DisplayStringAt(hist_x - 80, hist_y - hist_h / 2 - 6, (uint8_t*) half_period_buffer, LEFT_MODE);
	BSP_LCD_DisplayStringAt(hist_x - 80, hist_y - hist_h - 6, (uint8_t*) max_period_buffer, LEFT_MODE);
}

void render_graph(float *periods, uint16_t len) {
// Normalize data to histogram height
	float max_period = max_array_value(periods, 10);
	float min_period = min_array_value(periods, 10);
	printf("%f, %f\n\r", min_period, max_period);
	sprintf((char*) max_period_buffer, "%6.1f", max_period);
	sprintf((char*) half_period_buffer, "%6.1f", max_period - (max_period - min_period) / 2);
	float periods_normalized[10];
	normalize_array(periods, len, min_period, graph_h, max_period, periods_normalized);
// Draw histogram
	BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
	BSP_LCD_DrawLine(graph_x, graph_y, graph_x + graph_w, graph_y);
	BSP_LCD_DrawLine(graph_x, graph_y, graph_x, graph_y - graph_h);
	BSP_LCD_DrawDottedLine(graph_x, graph_y - graph_h / 2, graph_x + graph_w, graph_y - graph_h / 2, 8);
	BSP_LCD_DrawDottedLine(graph_x, graph_y - graph_h, graph_x + graph_w, graph_y - graph_h, 8);
	BSP_LCD_SetTextColor((uint32_t) 0xFFD9D9D9);
	int bin_w = graph_w / len;
//printf("%s\n\r", "Hist bars height values");
	for (uint16_t i = 0; i < len; i++) {
		BSP_LCD_FillCircle(1 + graph_x + i * bin_w, graph_y - periods_normalized[i] - 1, 3);
		//printf("%f\n\r", periods_normalized[i]);
	}
	BSP_LCD_SetFont(&LCD_FONT_12);
	BSP_LCD_DisplayStringAt(graph_x - 80, graph_y - graph_h / 2 - 6, (uint8_t*) half_period_buffer, LEFT_MODE);
	BSP_LCD_DisplayStringAt(graph_x - 80, graph_y - graph_h - 6, (uint8_t*) max_period_buffer, LEFT_MODE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_ResumeTick();
	if (GPIO_Pin == RAIN_Pin) Flag_EXTI15_RAIN = 1;
	if (GPIO_Pin == TOUCH_Pin) Flag_EXTI15_TOUCH = 1;
	if (GPIO_Pin == BTN_USER_Pin) Flag_EXTI11_BTN = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	HAL_ResumeTick();
	if (htim == &htim6) Flag_TIM6 = 1;
	if (htim == &htim7) Flag_TIM7 = 1;
}

/************** WIND SPEED ***************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	i++; //nombre d'impulsions (pour la moyenne)
	if (FIRST_IMP) {
		ccr0 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		FIRST_IMP = 0;
	}
	else {
		ccr1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		TIM1_IC_IT_Flag = 1;
	}

}

// calcul de la force du vent selon l'échelle de Beaufort
void WSpeed_To_WForce(float Wind_Speed, uint8_t *Force) {
//(wind_speed en MPH)
//passage par paramètre
	switch ((int) Wind_Speed) { //Case: [xmin ... xmax[
	case 0:
		*Force = 0; //Air calme
		strcpy((char*) wind_speed_beaufort_buffer, "Air calme");
		break;
	case 1 ... 3:
		*Force = 1; //Air léger
		strcpy((char*) wind_speed_beaufort_buffer, "Air léger");
		break;
	case 4 ... 7:
		*Force = 2; // Légère brise
		strcpy((char*) wind_speed_beaufort_buffer, "Légère brise");
		break;
	case 8 ... 12:
		*Force = 3; // Brise légère
		strcpy((char*) wind_speed_beaufort_buffer, "Brise légère");
		break;
	case 13 ... 17:
		*Force = 4; // Vent modéré
		strcpy((char*) wind_speed_beaufort_buffer, "Vent modéré");
		break;
	case 18 ... 24:
		*Force = 5; // Brise fraîche
		strcpy((char*) wind_speed_beaufort_buffer, "Brise fraîche");
		break;
	case 25 ... 30:
		*Force = 6; // Forte brise
		strcpy((char*) wind_speed_beaufort_buffer, "Forte brise");
		break;
	case 31 ... 38:
		*Force = 7; // Vent fort
		strcpy((char*) wind_speed_beaufort_buffer, "Vent fort");
		break;
	case 39 ... 46:
		*Force = 8; // Coup de vent
		strcpy((char*) wind_speed_beaufort_buffer, "Coup de vent");
		break;
	case 47 ... 54:
		*Force = 9; // Coup de vent de ficelle
		strcpy((char*) wind_speed_beaufort_buffer, "Coup de vent de ficelle");
		break;
	case 55 ... 63:
		*Force = 10; // Tempête
		strcpy((char*) wind_speed_beaufort_buffer, "Tempête");
		break;
	case 64 ... 73:
		*Force = 11; // Tempête violente
		strcpy((char*) wind_speed_beaufort_buffer, "Tempête violente");
		break;
	case 74 ... 1000:
		*Force = 12; // Ouragan
		strcpy((char*) wind_speed_beaufort_buffer, "Ouragan");
		break;
	}
}

/************** WIND DIR ************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	Wind_Dir_Voltage = HAL_ADC_GetValue(hadc);
	Wind_Dir_Flag = 1;
}

float deg_to_rad(float angle) {
	return angle * 2 * M_PI / 360;
}

void update_wind_dir(float angle_deg) {
	if (screen_index == WIND) {
		BSP_LCD_SetTextColor((uint32_t) 0xFF909090);
		float angle_offset = deg_to_rad(90);		//90° offset -> North = 0°
		float angle_rad = deg_to_rad(angle_deg) + angle_offset;
		uint16_t x = circle_x + circle_radius * -cos(angle_rad);
		uint16_t y = circle_y - circle_radius * sin(angle_rad);
		BSP_LCD_FillCircle(x, y, 7);
		BSP_LCD_SetBackColor((uint32_t) 0xFFF2F2F2);
		BSP_LCD_SetTextColor((uint32_t) 0xFF1E1E1E);
		BSP_LCD_DisplayStringAt(80, 100, (uint8_t*) wind_dir_angle_buffer, CENTER_MODE);
		BSP_LCD_DisplayStringAt(100, 130, (uint8_t*) dir_str, CENTER_MODE);
	}

}

/************** HUMIDITY ************/
void gethumidity() {
	/* Check device ID */

//	if (whoamI != HTS221_ID) {
//		hts221_status = HAL_ERROR;
//		printf("Could not find device ID %d (Humidity sensor) \n\r", HTS221_ID);
//	} else {
	hts221_status = HAL_OK;
	lin_t lin_hum;
	hts221_hum_adc_point_0_get(&dev_ctxHum, &lin_hum.x0);
	hts221_hum_rh_point_0_get(&dev_ctxHum, &lin_hum.y0);
	hts221_hum_adc_point_1_get(&dev_ctxHum, &lin_hum.x1);
	hts221_hum_rh_point_1_get(&dev_ctxHum, &lin_hum.y1);
	/* Enable Block Data Update */
	hts221_block_data_update_set(&dev_ctxHum, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	hts221_data_rate_set(&dev_ctxHum, HTS221_ODR_1Hz);
	/* Device power on */
	hts221_power_on_set(&dev_ctxHum, PROPERTY_ENABLE);
	memset(&data_raw_humidity, 0x00, sizeof(int16_t));
	hts221_humidity_raw_get(&dev_ctxHum, &data_raw_humidity);
	hum_perc = linear_interpolation(&lin_hum, data_raw_humidity);

	if (hum_perc < 0)
		hum_perc = 0;
	else if (hum_perc > 100) hum_perc = 100;

	if (first_hum) {
		hum_max_perc = hum_min_perc = hum_perc;
		first_hum = 0;
	}
	else {
		if (hum_perc >= hum_max_perc)
			hum_max_perc = hum_perc;
		else if (hum_perc <= hum_min_perc) hum_min_perc = hum_perc;
	}

	sprintf((char*) hum_buffer, "%5.1f", hum_perc);
	sprintf((char*) hum_min_buffer, "%5.1f", hum_min_perc);
	sprintf((char*) hum_max_buffer, "%5.1f", hum_max_perc);
	tx_com(tx_bufferH, strlen((char const*) tx_bufferH));
//}
}

/************** TEMPERATURE ************/
void gettemperature() {
	lps22hh_read_reg(&dev_ctx, LPS22HH_STATUS, (uint8_t*) &reg, 1);

	if (reg.status.t_da) {
		memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		lps22hh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
		temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature);
		if (!init_temp_min_max) {
			temperature_degCMin = temperature_degC;
			temperature_degCMax = temperature_degC;
			init_temp_min_max = 1;
		}
		sprintf((char*) temp_buffer, "%4.1f", temperature_degC);

		if (first_temp) {
			temperature_degCMax = temperature_degCMin = temperature_degC;
			first_temp = 0;
		}
		else {
			if (temperature_degC >= temperature_degCMax)
				temperature_degCMax = temperature_degC;
			else if (temperature_degC <= temperature_degCMin) temperature_degCMin = temperature_degC;
		}

		sprintf((char*) temp_max_buffer, "%4.1f", temperature_degCMax);
		sprintf((char*) temp_min_buffer, "%4.1f", temperature_degCMin);
		tx_com(temp_buffer, strlen((char const*) tx_buffer));
	}
}

/************** PRESSURE ************/
void getpression() {
	lps22hh_read_reg(&dev_ctx, LPS22HH_STATUS, (uint8_t*) &reg, 1);

	if (reg.status.p_da) {
		memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
		lps22hh_pressure_raw_get(&dev_ctx, &data_raw_pressure);
		pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure);

		if (!init_press_min_max) {
			pressure_min_hPa = pressure_hPa;
			pressure_max_hPa = pressure_hPa;
			init_press_min_max = 1;
		}

		if (first_press) {
			pressure_max_hPa = pressure_min_hPa = pressure_hPa;
			first_press = 0;
		}
		else {
			if (pressure_hPa >= pressure_max_hPa)
				pressure_max_hPa = pressure_hPa;
			else if (pressure_hPa <= pressure_min_hPa) pressure_min_hPa = pressure_hPa;
		}

		sprintf((char*) press_buffer, "%6.1f", pressure_hPa);
		sprintf((char*) press_max_buffer, "%6.1f", pressure_max_hPa);
		sprintf((char*) press_min_buffer, "%6.1f", pressure_min_hPa);
		tx_com(press_buffer, strlen((char const*) tx_bufferpress));
	}
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	HAL_I2C_Mem_Read(&hi2c1, LPS22HH_I2C_ADD_H, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
	HAL_I2C_Mem_Write(&hi2c1, LPS22HH_I2C_ADD_H, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

static int32_t platform_readH(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	reg |= 0x80;
	HAL_I2C_Mem_Read(&hi2c1, HTS221_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

static int32_t platform_writeH(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	reg |= 0x80;
	HAL_I2C_Mem_Write(&hi2c1, HTS221_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

void set_RGB_LED(enum LED color, GPIO_PinState state) {
	switch (color) {
	case RED:
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, state);
		break;
	case GREEN:
		HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, state);
		break;
	case BLUE:
		HAL_GPIO_WritePin(LD_B_GPIO_Port, LD_B_Pin, state);
		break;
	}
}

static void tx_com(uint8_t *tx_buffer, uint16_t len) {
	HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
