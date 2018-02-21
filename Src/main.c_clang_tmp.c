/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include <math.h>

#include "pcf8574.h"
#include "LiquidCrystal_I2C.h"
#include "tbolt.h"
#include "ad9852.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/

//#define __FPU_PRESENT 1
#define RTC_CLOCK_SOURCE_LSE

#define LCDADDR 0x27    // PCF8574T: If soldered - 0x20, if un-soldered - 0x27

#define ENCODERDELAY    10  // 1ms // 10mS // 512L;    // Delay before to check new value from Encoder
#define KNOPKADELAY     500 // 500mS

#define REDon()         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)
#define REDoff()        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
#define REDtgl()        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
#define GREENon()       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)
#define GREENoff()      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)
#define GREENtgl()      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

#define MULT_SW_PORT    GPIOE            // PE
#define MULT_SW_10x     GPIO_PIN_7
#define MULT_SW_100x    GPIO_PIN_8
#define MULT_SW_1000x   GPIO_PIN_10
#define MULT_SW_10000x  GPIO_PIN_11



#define MAX11205	18000000L	// 1.8V * 10^7
#define MAXDDSFREQ  80000000L   // Max DDS Frequency (10Mhz x 20m but Neuqist dictate 80Mhz)


// LCD string
uint8_t lcdstr[MAX_LCD_STRING+3];
__IO uint8_t tptr[TSBUF];

// PCF structure
PCF8574_HandleTypeDef	pcf;

// Encoder
uint8_t uwDirection = 1;
int32_t uwValue = 0L;

// Variable used to get converted value 
__IO uint16_t uhADCxConvertedValue = 0;


// Buttons
volatile uint16_t knopka_cnt = 0;
__IO uint8_t  knopka0 = 0;
__IO uint8_t  knopka15 = 0;

// RTC
RTC_TimeTypeDef stimestructureget;
RTC_DateTypeDef sdatestructureget;

// Alarm A flag
volatile uint8_t ALA = 0;

// SPI transfer states for AD9852 communication
__IO uint32_t wTransferState = TRANSFER_COMPLETE;
__IO uint32_t rTransferState = TRANSFER_COMPLETE;


// Extern
extern uint8_t syncf; 			// Its time to sync clocks
extern uint8_t utcf;		    // Guess: GPS or UTC ?
extern uint8_t sat_cnt;		    // Current Number of Visible Satelites


// Threading -----------------------------------------------

UBaseType_t uxHighWaterMark;

osThreadId DDS_threadHandle;
osThreadId TB_threadHandle;

osMutexDef (LCDmutexHandle);                        // Declare LCD mutex
osMutexId  LCDmutex;                                // Mutex ID

osMutexDef (PRNmutexHandle);                        // Declare PRN mutex
osMutexId  PRNmutex;                                // Mutex ID

//osMailQDef(TBmailHandle, 8, tb_msg);                // Define mail queue
//osMailQId  TBmail;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM12_Init(void);
void StartDefaultTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


extern uint32_t max_MAX11205_GPIO_SPI_read(uint8_t uchUseCalibrationMode);

void Error_Handler(void);


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// String/value convertors
uint16_t Sltoa(int64_t value, char* result, uint8_t base);
void LCDprint(uint8_t x, uint8_t y, char *str);
uint8_t BCDToDecimal (uint8_t bcdByte);
uint8_t DecimalToBCD (uint8_t decimalByte);
void StrPad(uint8_t l);
void printfcomma (int32_t n, char *str);
void format_commas(int32_t n, char *out);
uint16_t WeekDay(uint16_t CurrentYear, uint8_t CurrentMonth, uint8_t CurrentDay);

// uS delays
void DWT_Init(void);
uint32_t DWT_Get(void);
__inline uint32_t DWT_Compare(uint32_t t0, uint32_t t1);
void DWT_Delay(uint32_t us); // microseconds

// Threads
void StartDDS_thread(void const *p);
void StartTB_thread(void const *p);
void StartADC_thread(void const *p);
void StartTBswitch_thread(void const *p);

// Other
static void LCD_flash(uint8_t cnt, uint8_t delay);
uint16_t encoder_get_value(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

    uint8_t str[MAX_LCD_STRING+2];
    uint64_t fi = 0L;
    uint32_t esc1, esc2, esc3;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_TIM12_Init();

  /* USER CODE BEGIN 2 */

    // Fix the BUG
    if(!(*(volatile uint32_t *) (BDCR_RTCEN_BB)))__HAL_RCC_RTC_ENABLE();

    //if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    //    Error_Handler();
    //}

    // STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  
    if (HAL_GetREVID() == 0x1001) {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }

    printf("\r\n\r\n# Board has started with %d Hz Sytem Clock\r\n", SystemCoreClock);
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
        // SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access 
    printf("# FPU Initialized\r\n"); // Initialized
#endif

    
DWT_Init();     // Init uSec delays

    // Init PCF8574 I2C port extender (using in LCD interface)
    pcf.PCF_I2C_ADDRESS=7;
    pcf.PCF_I2C_TIMEOUT=1000;
    pcf.i2c.Instance=I2C1;
    pcf.i2c.Init.ClockSpeed=400000;

    if(PCF8574_Init(&pcf)!= PCF8574_OK){
        printf("# ERROR with PCF8574 port extension on LCD module\r\n");
        Error_Handler();
    } else {
        printf("# PCF8574 has been initialised\r\n");
    }

    LCDI2C_init(LCDADDR, 20, 4);        // LCD 20x4
    LCD_flash(3, 250); // ------- Quick 3 blinks of backlight  -------------
    LCDI2C_write_String("OSC: ");
    LCDI2C_setCursor(5, 0);
    Sltoa(SystemCoreClock, str, 10);
    LCDI2C_write_String(str);
    printf("# LCD module has been initialized\r\n");
    HAL_Delay(2000);
    LCDI2C_clear();

    // Encoder on Tim3
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    printf("# Encoder turned On\r\n");

    // Start Timer 12 [Timer-De-bouncer]
    // 1ms = 1000Hz, 1000Hz = 84Mhz/840000, 84000 too high (65536 is a MAX for 16 bit)
    // Prescaler = 2, then 84000 --> 420000, then Period = 42000-1 = 41999, and pulse will be 42000/2 = 21000 for 50% duty-cycles
    if(HAL_TIM_Base_Start_IT(&htim12) != HAL_OK) {
        printf("# Cannot start timer-debouncer [TIM12]\r\n");
        Error_Handler();
    } else {
        printf("# De-bouncer has been activated\r\n");
    }

    // Start ADC the conversion process #######################################
    if(HAL_ADC_Start_IT(&hadc2) != HAL_OK) {
        Error_Handler();
    }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  
    LCDmutex = osMutexCreate(osMutex(LCDmutexHandle));
    if (LCDmutex != NULL)  {
        printf("# LCD mutex has been created\r\n");
    }   

    PRNmutex = osMutexCreate(osMutex(PRNmutexHandle));
    if (PRNmutex != NULL)  {
        printf("# PRN mutex has been created\r\n");
    }   
  
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
 
    // DDS thread
    osThreadDef(DDS_thread, StartDDS_thread, osPriorityNormal, 1, 1024);    // 2048 | configMINIMAL_STACK_SIZE
    DDS_threadHandle = osThreadCreate(osThread(DDS_thread), NULL);

    // ThunderBolt  threads
    osThreadDef(TB_thread, StartTB_thread, osPriorityNormal, 1, 2048);                           // Assume that 2K will be enough for TB data
    TB_threadHandle = osThreadCreate(osThread(TB_thread), NULL);

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("# Default Thread HighWaterMark: %ld\n\r", uxHighWaterMark);
 
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
       
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc2);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* ETH init function */
void MX_ETH_Init(void)
{

   uint8_t MACAddr[6] ;

  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = 1;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_MII;
  HAL_ETH_Init(&heth);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 31;
  sDate.Year = 70;

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);

}

/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi3);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM12 init function */
void MX_TIM12_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 41999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&htim12);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 21000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : BLACK_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLACK_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLACK_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW0_Pin SW1_Pin SW2_Pin SW3_Pin 
                           PH_SW_Pin AM_SW_Pin */
  GPIO_InitStruct.Pin = SW0_Pin|SW1_Pin|SW2_Pin|SW3_Pin 
                          |PH_SW_Pin|AM_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin AD9852_CS_Pin AD9852_UCLK_Pin 
                           AD9852_RST_Pin AD9852_MRST_Pin AD9852OSK_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|AD9852_CS_Pin|AD9852_UCLK_Pin 
                          |AD9852_RST_Pin|AD9852_MRST_Pin|AD9852OSK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX11205_MISO_Pin */
  GPIO_InitStruct.Pin = MAX11205_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAX11205_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX11205_CLK_Pin */
  GPIO_InitStruct.Pin = MAX11205_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MAX11205_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_RED_Pin|AD9852_CS_Pin|AD9852_UCLK_Pin 
                          |MAX11205_CLK_Pin|AD9852_RST_Pin|AD9852_MRST_Pin|AD9852OSK_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* Flash RED LED */
    while(1) {
        REDtgl();
        HAL_Delay(127);
    }
}


void DWT_Init(void) 
{

  if( !( CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk ) || !( DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk ) ) {
    DWT->CYCCNT  = 0; 
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // enable the counter
  }
}
 
uint32_t DWT_Get(void)
{
  return DWT->CYCCNT;
}
 

__inline uint32_t DWT_Compare(uint32_t t0, uint32_t t1)
{
    return (t1 - t0); // always works, even if t1<t0
}

void DWT_Delay(uint32_t us) // microseconds (max = 2^32 / (fcpu*1.0E-6) - 1)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t dt = us * (SystemCoreClock/1000000UL); // 1us = 72tics @ 72MHz

    //printf("Delay: %lu:%lu\n", t0, dt);
    while (DWT_Compare(t0, DWT->CYCCNT) < dt)  {;}
}



/**
  * @brief  Blinks LCD
  * @param  counter
  * @retval None
  */
static void LCD_flash(uint8_t cnt, uint8_t delay)
{
    while(cnt--) {
        LCDI2C_backlight();
        HAL_Delay(delay);
        LCDI2C_noBacklight();
        HAL_Delay(delay);
    }
    LCDI2C_backlight(); // finish with backlight on
}


/**
 * print comma-separated numbers
 */
void format_commas(int32_t n, char *out)
{
    int32_t c;
    char buf[MAX_LCD_STRING];
    char *p;

    sprintf(buf, "%d", n);
    c = 2 - strlen(buf) % 3;
    for (p = buf; *p != 0; p++) {
       *out++ = *p;
       if (c == 1) {
           *out++ = ',';
       }
       c = (c + 1) % 3;
    }
    *--out = 0;
}


/**
 * print comma-separated numbers
 */
void printfcomma (int32_t n, char *str) 
{
    int32_t n2 = 0;
    int32_t scale = 1;
    uint8_t charbuf[MAX_LCD_STRING];


    if (n < 0) {
        str[0] = '-'; str[1] = '\0';
        n = -n;
    } else 
        str[0]='\0';

    while (n >= 1000) {
        n2 = n2 + scale * (n % 1000);
        n /= 1000;
        scale *= 1000;
    }
    sprintf(charbuf, "%d", n);
    strcat(str, charbuf);

    while (scale != 1) {
        scale /= 1000;
        n = n2 / scale;
        n2 = n2  % scale;
        sprintf(charbuf, ",%03d", n);
        strcat(str, charbuf);
    }
}

/**
    * SLTOA
    * Fast Long to ASCII converter
    * This function return length of string
 */
uint16_t Sltoa(int64_t value, char* result, uint8_t base) 
{
    char *ptr, *ptr1, tmp_char;
    int64_t tmp_value;
    uint16_t len = 0;

    // check that the base if valid
    if (base < 2 || base > 36) {
        *result = '\0'; 
        return 1;
    }
    ptr = ptr1 = result;
    do {
        len++;
        tmp_value = value;
        value /= base;
        //*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
        *ptr++ = "ZYXWVUTSRQPONMLKJIHGFEDCBA9876543210123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ" [35 + (tmp_value - value * base)];
    } while ( value );
	
    // Apply negative sign
    if (tmp_value < 0) {
        *ptr++ = '-';
        len++;
    }
    *ptr-- = '\0';
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return len+1;
}


// Padding string for LCD
void StrPad(uint8_t l)
{
    uint8_t i, flag = 0;

    for(i=0; i<l; i++) {
        if(!flag && lcdstr[i] == '\0')
            flag = 1;
        if(flag)
            lcdstr[i] = ' ';
    }
    lcdstr[i] = '\0';
}


// --------------- BCD COnvertors

uint8_t BCDToDecimal (uint8_t bcdByte)
{
  return (((bcdByte & 0xF0) >> 4) * 10) + (bcdByte & 0x0F);
}
 
uint8_t DecimalToBCD (uint8_t decimalByte)
{
  return (((decimalByte / 10) << 4) | (decimalByte % 10));
}


/**
  * @brief Determines the weekday
  * @param Year,Month and Day
  * @retval :Returns the CurrentWeekDay Number 1- Sunday 7- Saturday
  */
uint16_t WeekDay(uint16_t CurrentYear, uint8_t CurrentMonth, uint8_t CurrentDay)
{
    uint16_t Temp1,Temp2,Temp3,Temp4,CurrentWeekDay;
  
    if(CurrentMonth < 3) {
        CurrentMonth=CurrentMonth + 12;
        CurrentYear=CurrentYear-1;
    }
  
    Temp1=(6*(CurrentMonth + 1))/10;
    Temp2=CurrentYear/4;
    Temp3=CurrentYear/100;
    Temp4=CurrentYear/400;
    CurrentWeekDay=CurrentDay + (2 * CurrentMonth) + Temp1 \
        + CurrentYear + Temp2 - Temp3 + Temp4 +1;
    CurrentWeekDay = 1 + (CurrentWeekDay % 7);
  
    return(CurrentWeekDay);
}



// IRQ CallBAck functions

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    REDoff(); 
    // Disable the USART RXNE Interrupt 
    //__HAL_USART_DISABLE_IT(UartHandle, USART_IT_RXNE);
    //osSignalSet (TB_threadHandle, 0x00000005);  // dereference of RX_THREAD_POINTER
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  // Turn RED LED on: Transfer error in reception/transmission process 
  REDon(); 
}


/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and 
  *         you can add your own implementation.    
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  /* Get the converted value of regular channel */
  uhADCxConvertedValue = HAL_ADC_GetValue(AdcHandle);
}


/**
  * @brief  Alarm A callback
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    ALA = 1;
}	


/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt Tx transfer
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI3) {
        rTransferState = TRANSFER_COMPLETE;
        REDoff(); 
    }
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI3) {
        wTransferState = TRANSFER_COMPLETE;
        REDoff(); 
    }
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt Tx transfer
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI3) {
        wTransferState = TRANSFER_COMPLETE;
        REDoff(); 
    }
    
}


/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI3)
        rTransferState = wTransferState = TRANSFER_ERROR;
    // Turn RED LED on: Transfer error in reception/transmission process 
    REDon(); 
}


/**
  * @brief  PWM Pulse finished callback in non blocking mode
  * @param  htim : hadc handle
  * @retval None
  */
void __HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Timer 12. It sets to generate PWM 1000 Hz --> will overflow every milisecond
    // Its using as de-bouncer
    if (htim->Instance == TIM12) {
        if(knopka_cnt)
            --knopka_cnt;
        //if(enctick)  // commented out, as it not necessary for optical encoders
        //    --enctick;
    }    
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  *
  * Call coming from HAL_GPIO_EXTI_IRQHandler
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(!knopka_cnt) {
        knopka_cnt = KNOPKADELAY;
        if(GPIO_Pin == GPIO_PIN_2) 
            //if(HAL_GPIO_ReadPin(GPIOA, GPIO_Pin))
                knopka0 = 1;
    }    
}


/**
    * ENCODER
    * Read Encoder Value
    * This function return DELTA to add to FREQ.
**/
uint16_t encoder_get_value(void) {
  return (TIM3->CNT);
}



// --------------- Threads


// DDS Thread
void StartDDS_thread(void const *p) 
{
    uint16_t mult = 1, nEncoder = 0, oEncoder = 0, dEncoder = 0;
    uint8_t str[MAX_LCD_STRING];

    // Init DDS
    osMutexWait (PRNmutex, osWaitForever);
    AD9852_IO_Init();
    osMutexRelease (PRNmutex);

    while (1) {
        // Get the current encoder direction an value
        if( ((nEncoder = encoder_get_value()) != oEncoder) || ( ) {
            if(oEncoder >= nEncoder)
                dEncoder = (oEncoder - nEncoder);
            else
                dEncoder = (nEncoder - oEncoder);
            if(dEncoder > 0xF000)
                dEncoder = (uint16_t)((uint16_t)0xFFFF - dEncoder); // Filter zero-crossing and noise
            // Set Multiplier
            if(HAL_GPIO_ReadPin(MULT_SW_PORT, MULT_SW_1000x) == GPIO_PIN_SET) {
                mult = 1000;
            } else  if (HAL_GPIO_ReadPin(MULT_SW_PORT, MULT_SW_100x) == GPIO_PIN_SET) {
                    mult = 100;
                } else  if(HAL_GPIO_ReadPin(MULT_SW_PORT, MULT_SW_10x) == GPIO_PIN_SET) {
                        mult = 10;
                    } else {
                        mult = 1;
                    }
            dEncoder *= mult;
            // Set Value for FREQ
            if( (uwDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) ) {
                if( (uwValue += dEncoder) > MAXDDSFREQ || uwValue < 0)
                    uwValue = 0L;
            } else {
                if( (uwValue -= dEncoder) < 0 || uwValue > MAXDDSFREQ)
                    uwValue = MAXDDSFREQ;
            }
            oEncoder = nEncoder;

            format_commas(uwValue, str);
            osMutexWait (LCDmutex, osWaitForever);  // Wait indefinitely for a free semaphore
            sprintf(lcdstr, "DDS: %s Hz", str);
            StrPad(MAX_LCD_STRING);
            LCDI2C_setCursor(0, 2);			        // Second String for this packet (T-Bolt status)
            LCDI2C_write_String(lcdstr);
            osMutexRelease (LCDmutex);
            // Set Freq
            AD9852SetFreq1(uwValue);   

        } else { // Encoder not active
            osThreadYield();                                                    // Cooperative multitasking
        }
        osDelay(201);
    } // while(1)
}



/**
    * ThunderBolt Status Reading Thread function
    * Thread to parse T-Bolt messages
    * Its constanly retrieving TSIP data from T-BOLT RS232 port

    //#define ETX 0x03
    //#define DLE 0x10
    //0x10 0x8F 0xAB 0x0 0x3 0x92 0x88 0x6 0xF9 0x0 0x10 0x10 0x3 0x2C 0x1 0x11 0x19 0x3 0x7 0xDE 0x10 0x3 0x10
    //0x10 0x8F 0xAB 0x0 0x3 0xCC 0x16 0x6 0xF9 0x0 0x10 0x10 0x3 0x12 0x7 0x15 0x19 0x3 0x7 0xDE 0x10 0x3 0x10
    //<DLE> <id> <data string bytes> <DLE> <ETX>
 */
void StartTB_thread(void const *p) 
{
    uint8_t dle_active = 0;
    uint16_t i, ind = 0;
    uint8_t b;
    uint8_t sub, id;
    osEvent evt;

    // Loop through RS232 buffer a byte at a time, handling DLE state.
    //   1. DLE ETX  -- end of packet
    //   2. DLE <c>  -- escape c
    //   3. DLE      -- one of above
    //   4. <c>      -- normal data

    while(1) {
        // Signal flags that are reported as event are automatically cleared.
        //Thread::signal_wait(0x1);
        // Do cycle, then
        // LPC_UART0->IER = 1;               // Enable Rx interrupt
    
        // wait for a signal
        //evt = osSignalWait (0x00000005, osWaitForever);
        //if (evt.status != osEventSignal)  osThreadYield();
        if(HAL_UART_Receive_IT(&huart3, &b, 1) == HAL_OK) {
            if (dle_active) {
                if (b == ETX) {
                    // 1. End of packet detected (DLE ETX).
                    //printf("# Call tbSwitch --- 0x%X, 0x%X\r\n", tptr[0], tptr[1]);
                    GREENon();
                    // 1. End of packet detected (DLE ETX).
                    osMutexWait (LCDmutex, osWaitForever);                  // Wait indefinitely for a free semaphore
                    // Identify packets by packet id.
                    id = tptr[0]; sub = tptr[1];
                    //printf("\r\n# Call tbSwitch --- 0x%X, 0x%X\r\n", tptr[0], tptr[1]);
                    switch (id) {		              	                    // MSG ID (id)
                        case 0x6D : 
                                tsip_6D();
                                break;	        // Satellite Selection Packet (to count visible birds)
                        case 0x8F :
                                // Identify 8F packets by subcode.
                                switch (sub) {				// MSG SUB-ID
                                    case 0xAB : tsip_8F_AB(); break;	// Primary Timing Packet.
                                    case 0xAC : tsip_8F_AC(); break;	// Supplemental Timing Packet
                                    default :
                                    //printf("# GPS: ** 0x%.2X-%.2X TSIP packet not decoded\r\n", id, sub);
                                    break;
                                }
                                break;
                        default :
                                //printf("# GPS: ** 0x%.2X TSIP packet not decoded\r\n", id);
                                break;
                    }    
                    osMutexRelease (LCDmutex);
                    GREENoff();
                    dle_active = ind = 0;
                    //osDelay(100);
                } else {
                    // 2. Store escaped character.
                    tptr[ind++] = b;
                    if(ind>TSBUF) dle_active = ind = 0;
                }
                dle_active = 0;
            } else {
                if (b == DLE) {
                    // 3. Mark DLE state and get next character.
                    dle_active = 1;
                } else {
                    // 4. Store normal data byte.
                    tptr[ind++] = b;
                    if(ind>TSBUF) dle_active = ind = 0;
                }
            }
                
        } else {
            osThreadYield();
        }
    
    /* Calling the function uxTaskGetStackHighWaterMark() on top will have used some stack space, we would 
        therefore now expect uxTaskGetStackHighWaterMark() to return a 
        value lower than when it was called on entering the task. */
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //printf("Default Thread HighWaterMark: %ld\n\r", uxHighWaterMark);
    } // while (1)
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */

    uint32_t ADCoValue = 0L;         // Previous Averaged ADC value
    uint64_t fi = 0L;
    uint32_t esc1, esc2;

  /* Infinite loop */
  for(;;)
  {
  
    if(knopka0) {
        knopka0 = 0;
        GREENon();
        //Clock_Setup();
        //printf("# Clock setup has finished\r\n");
        GREENoff();
    }


    if(ALA) {
        osMutexWait (LCDmutex, osWaitForever);              // Wait indefinitely for a free semaphore
        HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
    	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
        if(utcf)
            sprintf(lcdstr, "UTC: %.2u:%.2u:%.2u S: %.2u", 
                stimestructureget.Hours,
                stimestructureget.Minutes,
                stimestructureget.Seconds,
                sat_cnt);
        else
            sprintf(lcdstr, "GPS: %.2u:%.2u:%.2u S: %.2u", 
                stimestructureget.Hours,
                stimestructureget.Minutes,
                stimestructureget.Seconds,
                sat_cnt);

        
        //printf("# %s\r\n", lcdstr);				    // DEBUG: GPS TIME
        
        if(uhADCxConvertedValue != ADCoValue) {                         // Nothing changed - skip and wait 10ms
            printf("AmpD: %lu\r\n", uhADCxConvertedValue);
            ADCoValue = uhADCxConvertedValue;
        }
    
        StrPad(MAX_LCD_STRING);
        LCDI2C_setCursor(0, 0);			    // First String for this packet (GPS/UTC time)
        LCDI2C_write_String(lcdstr);
        osMutexRelease (LCDmutex);
        if(!syncf && !stimestructureget.Minutes)
            if(!stimestructureget.Seconds && !(stimestructureget.Hours%4))
                syncf = 1;
        ALA = 0;
  }

    //Ready	eReady
    //Running	eRunning (the calling task is querying its own priority)
    //Blocked	eBlocked
    //Suspended	eSuspended
    //Deleted	eDeleted (the tasks TCB is waiting to be cleaned up)
    if (eTaskGetState(TB_threadHandle) != eReady) {
        printf("# T-Bolt thread is not ready\r\n");
    }


    // Get Phase Difference and calculate the degree
    // Note: t = fi/(360 *frq), where fi - is phase diff in degree.
    if( (fi = MAX11205 * (uint64_t) max_MAX11205_GPIO_SPI_read(0)) ) {
        fi = (fi>>16);
        esc1 = (uint32_t)(fi/100000L);
        esc2 = (uint32_t)(fi%100000L);
        if(esc1 || esc2) {
            osMutexWait (LCDmutex, osWaitForever);              // Wait indefinitely for a free semaphore
            //printf("ADC: %lu.%lu degree\r\n", esc1, esc2);
            sprintf(lcdstr, "PhD: %lu.%lu dg", esc1, esc2);
            StrPad(MAX_LCD_STRING);
            //printf("# %s\r\n", lcdstr);
            LCDI2C_setCursor(0, 3);			                    // Last string for Phase Differences
            LCDI2C_write_String(lcdstr);
            osMutexRelease (LCDmutex);
        }
    }

    osDelay(300);
  } // for(;;)

  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
