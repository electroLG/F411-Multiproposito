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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BKP_REG.h"
#include "string.h"
#include "stdio.h"
#include "ESP8266_Chelo.h"
#include "ModBUS_Chelo.h"
#include "STR_Chelo.h"
#include "ETH_W5100.h"
#include "http.h"
#include "string.h"
#include "RYLR896.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

struct LoRa lr;
struct WIFI wf;
struct W5100_SPI ETH;
struct MBUS mb_lr;

uint32_t dataRTC[20];

int		mseg=0; //conteo de milisegundos

char 	WIFI_NET[]="PLC_DEV",					//WIFI_NET[]="Fibertel WiFi967 2.4GHz",//WIFI_NET[]="PLC_DEV",//
		WIFI_PASS[]="12345678",					//WIFI_PASS[]="0042880756",//WIFI_PASS[]="12345678",//
		TCP_SERVER[]="192.168.0.91",			//TCP_SERVER[]="192.168.0.65",//TCP_SERVER[]="192.168.0.102",//TCP_SERVER[]="192.168.0.47",
		TCP_PORT[]="8000",						//TCP_PORT[]="502",
		TCP_SERVER_LOCAL[]="192.168.0.33",		//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_GWY[]="192.168.0.99",	//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_MSK[]="255.255.255.0",	//TCP_SERVER[]="192.168.0.47",
		TCP_PORT_LOCAL[]="502",

		READ_FUNCTION_1[16],
		READ_FUNCTION_2[16],

		READ_HLF_FUNC_1[8],
		READ_HLF_FUNC_2[8];

/*char post[512];
char body[512];
char ENDPOINT[]="/logdata",//ENDPOINT[]="/tepelco",
     SERVER_IP[]="192.168.0.91",
     PORT[]="8000";*/

uint8_t EN_UART1_TMR=0,
		EN_UART2_TMR=0,
		EN_UART6_TMR=0,
		FLAG_UART1=0,
		FLAG_UART2=0,
		FLAG_UART6=0;
		/*decimal[17],
		error=0,
		ESP_REinit=0,			//Conteo de intentos de incializacion
		ESP_InitF=0,			//Flag de error por no encontrar la sentencia
		ESP_HW_Init=0,
		FLAG_TIMEOUT=0,
		resultado=0,
		error_rxdata=0,
		esp_restart=0,
		conexion,
		WF_SND_FLAG=0;*/

//uint16_t datos[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*uint32_t ms_ticks=0,
		 min_ticks=0;*/


char	UART_RX_vect[1024],
		UART2_RX_vect[512],
		UART6_RX_vect[512],
		UART_RX_vect_hld[1024],
		CMP_VECT[]="\0",
		UART_RX_byte[2],
		UART2_RX_byte[2],
		UART6_RX_byte[2];

int 	//wf_snd_flag_ticks=0,
		UART_RX_items=0,
		UART2_RX_items=0,
		UART6_RX_items=0,
		UART_RX_pos=0,
		UART2_RX_pos=0,
		UART6_RX_pos=0;
		//ESP_ticks=0,
		//MB_TOUT_ticks=0,
		//ticks=0,
		//ntestc=0,
		//uart1pass=0,
		//USART1_ticks=0,
		//FLAG_USART1=0,
		//items_rx=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

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


	  //----------------------- WIFI ------------------------//
	  	Inicializar(&wf); 									//Borra todos los registros de la estructura
	  	wf.RESET_PORT=GPIOA;
	  	wf.RESET_PIN=GPIO_PIN_8;
		strcpy(wf._WF_Net, WIFI_NET);						//Nombre de la red WIFI  a conectar Fibertel WiFi967 2.4GHz
		strcpy(wf._WF_Pass, WIFI_PASS);						//Password de la red WIFI
		strcpy(wf._TCP_Remote_Server_IP, TCP_SERVER);		//char _TCP_Remote_Server_IP[16];		//IP del Servidor TCP
		strcpy(wf._TCP_Remote_Server_Port, TCP_PORT);		//char _TCP_Remote_Server_Port[16];			//Puerto del Servidor TCP
		strcpy(wf._TCP_Local_Server_IP, TCP_SERVER_LOCAL);
		strcpy(wf._TCP_Local_Server_GWY, TCP_SERVER_LOCAL_GWY);
		strcpy(wf._TCP_Local_Server_MSK, TCP_SERVER_LOCAL_MSK);
		strcpy(wf._TCP_Local_Server_Port, TCP_PORT_LOCAL);
		wf._TCP_Local_Server_EN=0;							//Habilito el Servidor Local
		wf._estado_conexion=100;//Si no se define no arranca	//wf._estado_conexion=1;					//Arranco en WiFi Desconectado
		wf._automatizacion=WF_CONNECT_TCP;//wf._automatizacion=WF_SEND;
		wf._NO_IP=1;
		wf._DBG_EN=1;
	//----------------------- WIFI ------------------------//
	//---------------------- ModBUS -----------------------//

		ModBUS_Config(&mb_lr);		//ETHERNET como cliente TCP envía  ModBUS
		mb_lr._mode = CLIENTE;

	//---------------------- ModBUS -----------------------//
	//----------------------- ETHERNET W5100 Environment-------------------------//

	//	GATEWAY ADDRESS
		ETH.GAR[0]=192;
		ETH.GAR[1]=168;
		ETH.GAR[2]=0;
		ETH.GAR[3]=1;
	//	SUBNET MASK
		ETH.SUBR[0]=255;
		ETH.SUBR[1]=255;
		ETH.SUBR[2]=255;
		ETH.SUBR[3]=0;
	//	MAC ADDRESS
		ETH.SHAR[0]=0x00;
		ETH.SHAR[1]=0x08;
		ETH.SHAR[2]=0xDC;
		ETH.SHAR[3]=0x00;
		ETH.SHAR[4]=0x00;
		ETH.SHAR[5]=0x01;
	//	IP ADDRESS
		ETH.SIPR[0]=192;
		ETH.SIPR[1]=168;
		ETH.SIPR[2]=0;
		ETH.SIPR[3]=34;//ETH.SIPR[3]=6,
	//  Socket RX memory
		ETH.RMSR=0x55;
	//  Socket TX memory
		ETH.TMSR=0x55;
	//  S0 Port Number
		ETH.S0_PORT[0]=0x01;
		ETH.S0_PORT[1]=0xF6;
	//	S0 Client IP ADDRESS
		ETH.S0_DIPR[0]=192;
		ETH.S0_DIPR[1]=168;
		ETH.S0_DIPR[2]=0;
		ETH.S0_DIPR[3]=3;//=3;
	//	S0 Client IP Port
		ETH.S0_DPORT[0]=0x01;
		ETH.S0_DPORT[1]=0xF6;

		ETH.gS0_RX_BASE = 0x6000;
		ETH.gS0_RX_MASK = 0x07FF;
		ETH.gS1_RX_BASE = 0x6800;
		ETH.gS1_RX_MASK = 0x07FF;
		ETH.gS2_RX_BASE = 0x7000;
		ETH.gS2_RX_MASK = 0x07FF;
		ETH.gS3_RX_BASE = 0x7800;
		ETH.gS3_RX_MASK = 0x07FF;
		ETH.gS0_TX_BASE = 0x4000;
		ETH.gS0_TX_MASK = 0x07FF;
		ETH.gS1_TX_BASE = 0x4800;
		ETH.gS1_TX_MASK = 0x07FF;
		ETH.gS2_TX_BASE = 0x5000;
		ETH.gS2_TX_MASK = 0x07FF;
		ETH.gS3_TX_BASE = 0x5800;
		ETH.gS3_TX_MASK = 0x07FF;

		ETH.S0_ENserver = 0;			//Actúa como servidor S0_ENserver=1 o cliente S0_ENserver=0

	//----------------------- ETHERNET W5100 Environment-------------------------//



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
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  ITM0_Write("\r\n ARRANQUE",strlen("\r\n ARRANQUE"));

  dataRTC[3]=128;
  BKP_REG_blk(&hrtc, WRITE , 20, dataRTC );
  dataRTC[3]=0;
  //BKP_REG_blk(&hrtc, READ , 20, dataRTC );

  BKP_REG_blk(&hrtc, READ , 20, dataRTC );

  BKP_RG_IP(&hrtc, WRITE, 0, TCP_SERVER);
  BKP_RG_IP(&hrtc, WRITE, 1, TCP_SERVER_LOCAL);
  BKP_RG_IP(&hrtc, WRITE, 3, TCP_SERVER_LOCAL_GWY);
  BKP_RG_IP(&hrtc, WRITE, 4, TCP_SERVER_LOCAL_MSK);

  BKP_RG_IP(&hrtc, READ, 0, READ_FUNCTION_1);
  BKP_RG_IP(&hrtc, READ, 1, READ_FUNCTION_2);

  BKP_RG_2int(&hrtc, WRITE , 2 , TCP_PORT ,TCP_PORT_LOCAL );
  BKP_RG_2int(&hrtc, READ , 2 , READ_HLF_FUNC_1 ,READ_HLF_FUNC_2 );

  if(dataRTC[3]==128)
  {
	  ITM0_Write("\r\n Valor guardado en RTC ",strlen("\r\n Valor guardado en RTC "));
  }

  HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1);
  HAL_UART_Receive_IT(&huart2,(uint8_t *)UART2_RX_byte,1);
  HAL_UART_Receive_IT(&huart6,(uint8_t *)UART6_RX_byte,1);
  ITM0_Write("\r\nPuerto serie en escucha",strlen("\r\nPuerto serie en escucha"));


	//------------------ Habilitacion de dispositivos ---------------//

	  HAL_GPIO_WritePin(GPIOB, LR_RST_Pin, GPIO_PIN_SET);		//Habilito LoRa
	  HAL_GPIO_WritePin(GPIOB, MBUS_CTRL_Pin, GPIO_PIN_RESET);	//Habilito 485 para RX
	  HAL_GPIO_WritePin(GPIOA, WF_EN_RST_Pin, GPIO_PIN_SET);	//Habilito WiFi


	//------------------ Habilitacion de dispositivos ---------------//


  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 150;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 150;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Q0_0_GPIO_Port, Q0_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Q0_1_Pin|MBUS_CTRL_Pin|WF_EN_RST_Pin|DBG_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FALLA_Pin|CNN_Pin|ALIM_Pin|ETH_NSS_Pin
                          |ETH_RST_Pin|LR_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Q0_0_Pin */
  GPIO_InitStruct.Pin = Q0_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Q0_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Q0_1_Pin MBUS_CTRL_Pin WF_EN_RST_Pin DBG_PIN_Pin */
  GPIO_InitStruct.Pin = Q0_1_Pin|MBUS_CTRL_Pin|WF_EN_RST_Pin|DBG_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A3V3_Pin A5V_Pin IA0_Pin IA1_Pin */
  GPIO_InitStruct.Pin = A3V3_Pin|A5V_Pin|IA0_Pin|IA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FALLA_Pin CNN_Pin ALIM_Pin ETH_NSS_Pin
                           ETH_RST_Pin LR_RST_Pin */
  GPIO_InitStruct.Pin = FALLA_Pin|CNN_Pin|ALIM_Pin|ETH_NSS_Pin
                          |ETH_RST_Pin|LR_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
   mseg++;

   if (mseg==500)
   {
	   HAL_GPIO_TogglePin(GPIOB, FALLA_Pin);
	   HAL_GPIO_TogglePin(GPIOB, CNN_Pin);
	   HAL_GPIO_TogglePin(GPIOB, ALIM_Pin);
	   mseg=0;

   }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *INTSERIE)
{

// WiFi	USART 1 TIMER2
	if(INTSERIE->Instance==USART1)
		 {
			UART_RX_vect[UART_RX_pos]=UART_RX_byte[0];
			UART_RX_pos++;
			if(UART_RX_pos>=1022) UART_RX_pos=1022;
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//HAL_TIM_Base_Start_IT(&htim7);	//Habilito el timer
			TIM2->CNT=1;
			EN_UART1_TMR=1;	//Habilito Timeout de software
			HAL_UART_Receive_IT(INTSERIE,(uint8_t *)UART_RX_byte,1);
		 }
// MBUS USART2 TIMER3
	if(INTSERIE->Instance==USART2)
		 {
			UART2_RX_vect[UART2_RX_pos]=UART2_RX_byte[0];
			UART2_RX_pos++;
			if(UART2_RX_pos>=512) UART2_RX_pos=512;
			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);//HAL_TIM_Base_Start_IT(&htim7);	//Habilito el timer
			TIM3->CNT=1;
			EN_UART2_TMR=1;	//Habilito Timeout de software
			HAL_UART_Receive_IT(INTSERIE,(uint8_t *)UART2_RX_byte,1);
		 }
// LoRa/debgg USART6 TIMER3
	if(INTSERIE->Instance==USART6)
		 {
			UART6_RX_vect[UART6_RX_pos]=UART6_RX_byte[0];
			UART6_RX_pos++;
			if(UART6_RX_pos>=512) UART6_RX_pos=512;
			HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);//HAL_TIM_Base_Start_IT(&htim7);	//Habilito el timer
			TIM4->CNT=1;
			EN_UART6_TMR=1;	//Habilito Timeout de software
			HAL_UART_Receive_IT(INTSERIE,(uint8_t *)UART6_RX_byte,1);
		 }
 }

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *TIMER)
{
// WiFi	USART 1 TIMER2
		if(TIMER->Instance==TIM2)
			{
				 HAL_TIM_OC_Stop_IT(TIMER, TIM_CHANNEL_1); //Paro el timer
				 FLAG_UART1=1;
				 EN_UART1_TMR=0;
				 UART_RX_items=UART_RX_pos;
				 UART_RX_pos=0;
				 UART_RX_vect[1022]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(UART_RX_vect_hld,UART_RX_vect,UART_RX_items,1,CMP_VECT);
				 HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1); //Habilito le recepcón de puerto serie al terminar
				 if (wf._DBG_EN==1)
				 {
					 ITM0_Write("\r\nData WIFI recibida = ",strlen("\r\nData WIFI recibida = "));
					 ITM0_Write((uint8_t *)UART_RX_vect_hld,UART_RX_items);
					 ITM0_Write("\r\n",strlen("\r\n"));
				 }
		}
// MBUS USART2 TIMER3
		if(TIMER->Instance==TIM3)
			{
				 HAL_TIM_OC_Stop_IT(TIMER, TIM_CHANNEL_1); //Paro el timer
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				 FLAG_UART2=1;
				 EN_UART2_TMR=0;
				 UART2_RX_items=UART2_RX_pos;
				 UART2_RX_pos=0;
				 UART2_RX_vect[512]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(lr.rxbuff,UART2_RX_vect,UART2_RX_items,1,CMP_VECT);
				 lr.rxitems=UART2_RX_items;
				 HAL_UART_Receive_IT(&huart2,(uint8_t *)UART2_RX_byte,1); //Habilito le recepcón de puerto serie al terminar
				 if (wf._DBG_EN==1)
				 {
					 ITM0_Write("\r\nData MBUS recibida = ",strlen("\r\nData MUS recibida = "));
					 ITM0_Write((uint8_t *)UART2_RX_vect,UART2_RX_items);
					 ITM0_Write("\r\n",strlen("\r\n"));
				 }
		}

// LoRa/Debbg USART6 TIMER3
		if(TIMER->Instance==TIM4)
			{
				 HAL_TIM_OC_Stop_IT(TIMER, TIM_CHANNEL_1); //Paro el timer
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				 FLAG_UART6=1;
				 EN_UART6_TMR=0;
				 UART6_RX_items=UART6_RX_pos;
				 UART6_RX_pos=0;
				 UART6_RX_vect[512]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(lr.rxbuff,UART2_RX_vect,UART2_RX_items,1,CMP_VECT);
				 lr.rxitems=UART2_RX_items;
				 HAL_UART_Receive_IT(&huart2,(uint8_t *)UART2_RX_byte,1); //Habilito le recepcón de puerto serie al terminar
				 if (wf._DBG_EN==1)
				 {
					 ITM0_Write("\r\nData LoRa recibida = ",strlen("\r\nData LoRa recibida = "));
					 ITM0_Write((uint8_t *)UART6_RX_vect,UART6_RX_items);
					 ITM0_Write("\r\n",strlen("\r\n"));
				 }
		}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *ERRUART)

{
	if(ERRUART->Instance==USART1)
	{
		 volatile int aore=0;
		 volatile int bore=0;
		//Al leer los registros de esta forma SR y luego DR se resetean los errores de Framing Noise y Overrun FE NE ORE
		 aore=ERRUART->Instance->SR;
		 bore=ERRUART->Instance->DR;
		 HAL_UART_DeInit(ERRUART);
		 MX_USART1_UART_Init();
		 HAL_UART_Receive_IT(ERRUART,(uint8_t *)UART_RX_byte,1);
	}
	if(ERRUART->Instance==USART2)
	{
		 volatile int aore=0;
		 volatile int bore=0;
		//Al leer los registros de esta forma SR y luego DR se resetean los errores de Framing Noise y Overrun FE NE ORE
		 aore=ERRUART->Instance->SR;
		 bore=ERRUART->Instance->DR;
		 HAL_UART_DeInit(ERRUART);
		 MX_USART2_UART_Init();
		 HAL_UART_Receive_IT(ERRUART,(uint8_t *)UART2_RX_byte,1);
	}
	if(ERRUART->Instance==USART6)
	{
		 volatile int aore=0;
		 volatile int bore=0;
		//Al leer los registros de esta forma SR y luego DR se resetean los errores de Framing Noise y Overrun FE NE ORE
		 aore=ERRUART->Instance->SR;
		 bore=ERRUART->Instance->DR;
		 HAL_UART_DeInit(ERRUART);
		 MX_USART6_UART_Init();
		 HAL_UART_Receive_IT(ERRUART,(uint8_t *)UART6_RX_byte,1);
	}

}

int ITM0_Write( char *ptr, int len)
{
 int DataIdx;

  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
