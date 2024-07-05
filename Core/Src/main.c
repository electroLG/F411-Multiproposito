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
#define UID_BASE_ADDRESS 0x1FFF7A10U
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
struct MBUS mb_eth;			// Instancia MODBus Ethernet
struct MBUS mb_wf;			// Instancia MODBus WiFi
struct BKP_REG NVS;

uint32_t dataRTC[20],
		 UID[3];

int		mseg=0, //conteo de milisegundos
	   	ms_ticks=0,
	   	min_ticks=0,
		MB_TOUT_ticks=0;

char 	WIFI_NET[]="PLC_DEV_CON_TI",					//WIFI_NET[]="Fibertel WiFi967 2.4GHz",//WIFI_NET[]="PLC_DEV",//
		WIFI_PASS[]="123456789",					//WIFI_PASS[]="0042880756",//WIFI_PASS[]="12345678",//
		WIFI__IP[]="192.168.0.33",
		WIFI_MASK[]="255.255.255.0",

		EP_SERVER[]="192.168.0.91",			//TCP_SERVER[]="192.168.0.65",//TCP_SERVER[]="192.168.0.102",//TCP_SERVER[]="192.168.0.47",
		EP_PORT[]="8000",						//TCP_PORT[]="502",

		TCP_SERVER_LOCAL[]="192.168.5.2",		//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_GWY[]="192.168.5.1",	//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_MSK[]="255.255.255.0",	//TCP_SERVER[]="192.168.0.47",
		TCP_PORT_LOCAL[]="8000",

		ETHERNET_IP[]="192.168.0.44",
		ETHERNET_TRGT_IP[]="192.168.0.3",
		ETHERNET_MASK[]="255.255.255.0",
		ETHERNET_PORT[]="502",

		LORA_ADDR[]="3",
		LORA_NET_ID[]="1",
		LORA_NCPIN[]="3",
		LORA_BAND[]="55",

		MBUS_REG[]="16",
		MBUS_ID[]="1",
		MBUS_CODE[]="4",
		MBUS_SRVR[]="2",


		READ_FUNCTION_1[16],
		READ_FUNCTION_2[16],
		READ_FUNCTION_3[16],
		READ_FUNCTION_4[16],
		READ_FUNCTION_5[16],
		READ_FUNCTION_6[16],
		READ_FUNCTION_7[16],
		READ_FUNCTION_8[16],
		READ_FUNCTION_9[16],
		READ_FUNCTION_10[16],
		READ_FUNCTION_11[16],
		READ_FUNCTION_12[16],
		READ_FUNCTION_13[16],
		READ_FUNCTION_14[16],
		READ_FUNCTION_15[16],
		READ_FUNCTION_16[16],

		READ_HLF_FUNC_1[8],
		READ_HLF_FUNC_2[8];

/*char post[512];
char body[512];
char ENDPOINT[]="/logdata",//ENDPOINT[]="/tepelco",
     SERVER_IP[]="192.168.0.91",
     PORT[]="8000";*/
uint16_t 	S0_get_size = 0,
			tx_mem_pointer=0,
			rx_mem_pointer=0,
			send_size=0;

uint8_t EN_UART1_TMR=0,
		EN_UART2_TMR=0,
		EN_UART6_TMR=0,
		FLAG_UART1_WF=0,
		FLAG_UART2_485=0,
		FLAG_UART6=0,
		SYS_WEB_SERVER=0,
		SYS_DEBUG_EN=1,
		SYS_SPI_ETH_READ_EN=0,
		SYS_ETH_DBG_EN=1,
		SYS_configData=0,
		spi_no_debug=0,
		spi_Data[64];

char	UART_RX_vect[4096],//UART_RX_vect[1024],
		UART2_RX_vect[512],
		UART6_RX_vect[4096],//UART6_RX_vect[512],
		UART_RX_vect_hld[4096],//UART_RX_vect_hld[1024],
		UART2_RX_vect_hld[1024],
		UART6_RX_vect_hld[4096],//UART6_RX_vect_hld[1024],
		CMP_VECT[]="\0",
		UART_RX_byte[2],
		UART2_RX_byte[2],
		UART6_RX_byte[2],
		APWFUID[65],
		dummyArray[20],
    	WEB[]="<!DOCTYPE html><html><body><h2>Datos salvados</h2></body></html>\r\n",
		WEB2[]="<!DOCTYPE html><html><body><h2>SetUp RIoT device</h2><form action=\"/192.168.0.14:80\"><p>IP SRVR</p><input type=\"number\" id=\"A\" name=\"A\" min=\"1\" max=\"254\"><input type=\"number\" id=\"B\" name=\"B\" min=\"1\" max=\"254\"><input type=\"number\" id=\"C\" name=\"C\" min=\"1\" max=\"254\"><input type=\"number\" id=\"D\" name=\"D\" min=\"1\" max=\"254\"><p>EP KEY</p><input type=\"number\" id=\"E\" name=\"E\" min=\"1\" max=\"4095\"><p>ETH IP</p><input type=\"number\" id=\"F\" name=\"F\" min=\"1\" max=\"254\"><input type=\"number\" id=\"G\" name=\"G\" min=\"1\" max=\"254\"><input type=\"number\" id=\"H\" name=\"H\" min=\"1\" max=\"254\"><input type=\"number\" id=\"I\" name=\"I\" min=\"1\" max=\"254\"><p>ETH MSK</p><input type=\"number\" id=\"J\" name=\"J\" min=\"1\" max=\"254\"><input type=\"number\" id=\"K\" name=\"K\" min=\"1\" max=\"254\"><input type=\"number\" id=\"L\" name=\"L\" min=\"1\" max=\"254\"><input type=\"number\" id=\"M\" name=\"M\" min=\"1\" max=\"254\"><p>ETH PRT</p><input type=\"number\" id=\"N\" name=\"N\" min=\"1\" max=\"65535\"><p>WF ID</p><input type=\"text\" id=\"O\" name=\"O\" maxlength=\"28\" size=\"28\"><p>WF PSS</p><input type=\"password\" id=\"P\" name=\"P\" maxlength=\"12\" size=\"12\"><p>WF IP</p><input type=\"number\" id=\"Q\" name=\"Q\" min=\"1\" max=\"254\"><input type=\"number\" id=\"R\" name=\"R\" min=\"1\" max=\"254\"><input type=\"number\" id=\"S\" name=\"S\" min=\"1\" max=\"254\"><input type=\"number\" id=\"T\" name=\"T\" min=\"1\" max=\"254\"><p>WF MSK</p><input type=\"number\" id=\"U\" name=\"U\" min=\"1\" max=\"254\"><input type=\"number\" id=\"V\" name=\"V\" min=\"1\" max=\"254\"><input type=\"number\" id=\"W\" name=\"W\" min=\"1\" max=\"254\"><input type=\"number\" id=\"X\" name=\"X\" min=\"1\" max=\"254\"><p>WF ORT</p><input type=\"number\" id=\"Y\" name=\"Y\" min=\"1\" max=\"65535\"><p>LoRa ADDR</p><input type=\"number\" id=\"Z\" name=\"Z\" min=\"1\" max=\"254\"><p>LoRa NID</p><input type=\"number\" id=\"0\" name=\"0\" min=\"0\" max=\"16\"><p>LoRa NCP</p><input type=\"number\" id=\"1\" name=\"1\" min=\"0\" max=\"16\"><p>LoRa BND</p><input type=\"number\" id=\"2\" name=\"2\" min=\"0\" max=\"16\"><input type=\"submit\" value=\"OK\"></form></body></html>\r\n",
		WEB3[]="<!DOCTYPE html><html><body><h2>SetUp RIoT device</h2><form action=\"/192.168.4.1:80\"><p>IP SRVR</p><input type=\"number\" id=\"A\" name=\"A\" min=\"1\" max=\"254\"><input type=\"number\" id=\"B\" name=\"B\" min=\"0\" max=\"255\"><input type=\"number\" id=\"C\" name=\"C\" min=\"0\" max=\"255\"><input type=\"number\" id=\"D\" name=\"D\" min=\"1\" max=\"254\"><p>EP KEY</p><input type=\"number\" id=\"E\" name=\"E\" min=\"1\" max=\"4095\"><p>ETH IP</p><input type=\"number\" id=\"F\" name=\"F\" min=\"1\" max=\"255\"><input type=\"number\" id=\"G\" name=\"G\" min=\"0\" max=\"255\"><input type=\"number\" id=\"H\" name=\"H\" min=\"0\" max=\"255\"><input type=\"number\" id=\"I\" name=\"I\" min=\"1\" max=\"254\"><p>ETH MSK</p><input type=\"number\" id=\"J\" name=\"J\" min=\"0\" max=\"255\"><input type=\"number\" id=\"K\" name=\"K\" min=\"0\" max=\"255\"><input type=\"number\" id=\"L\" name=\"L\" min=\"0\" max=\"255\"><input type=\"number\" id=\"M\" name=\"M\" min=\"0\" max=\"254\"><p>ETH PRT</p><input type=\"number\" id=\"N\" name=\"N\" min=\"1\" max=\"65535\">\r\n",
		WEB4[]="<p>WF ID</p><input type=\"text\" id=\"O\" name=\"O\" maxlength=\"28\" size=\"28\"><p>WF PSS</p><input type=\"password\" id=\"P\" name=\"P\" maxlength=\"12\" size=\"12\"><p>WF IP</p><input type=\"number\" id=\"Q\" name=\"Q\" min=\"1\" max=\"255\"><input type=\"number\" id=\"R\" name=\"R\" min=\"0\" max=\"255\"><input type=\"number\" id=\"S\" name=\"S\" min=\"0\" max=\"255\"><input type=\"number\" id=\"T\" name=\"T\" min=\"1\" max=\"254\"><p>WF MSK</p><input type=\"number\" id=\"U\" name=\"U\" min=\"0\" max=\"255\"><input type=\"number\" id=\"V\" name=\"V\" min=\"0\" max=\"255\"><input type=\"number\" id=\"W\" name=\"W\" min=\"0\" max=\"255\"><input type=\"number\" id=\"X\" name=\"X\" min=\"0\" max=\"254\"><p>WF PRT</p><input type=\"number\" id=\"Y\" name=\"Y\" min=\"1\" max=\"65535\"><p>LoRa ADDR</p><input type=\"number\" id=\"Z\" name=\"Z\" min=\"1\" max=\"254\"><p>LoRa NID</p><input type=\"number\" id=\"0\" name=\"0\" min=\"0\" max=\"16\"><p>LoRa NCP</p><input type=\"number\" id=\"1\" name=\"1\" min=\"0\" max=\"16\"><p>LoRa BND</p><input type=\"number\" id=\"2\" name=\"2\" min=\"0\" max=\"158\"><input type=\"submit\" value=\"OK\"></form></body></html>\r\n";
int 	//wf_snd_flag_ticks=0,
		dummy=0,
		dummy2=3,
		dummy3=27,
		dummy4=5,
		UART_RX_items=0,
		UART2_RX_items=0,
		UART6_RX_items=0,
		UART_RX_pos=0,
		UART2_RX_pos=0,
		UART6_RX_pos=0;

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


	  //-----------------------1 WIFI ------------------------//
	  	Inicializar(&wf); 									//Borra todos los registros de la estructura
	  	wf.RESET_PORT=GPIOA;
	  	wf.RESET_PIN=GPIO_PIN_8;
		strcpy(wf._WF_Net, WIFI_NET);						//Nombre de la red WIFI  a conectar Fibertel WiFi967 2.4GHz
		strcpy(wf._WF_Pass, WIFI_PASS);						//Password de la red WIFI
		strcpy(wf._TCP_Remote_Server_IP, EP_SERVER);		//char _TCP_Remote_Server_IP[16];		//IP del Servidor TCP
		strcpy(wf._TCP_Remote_Server_Port, EP_PORT);		//char _TCP_Remote_Server_Port[16];			//Puerto del Servidor TCP
		strcpy(wf._TCP_Local_Server_IP, TCP_SERVER_LOCAL);
		strcpy(wf._TCP_Local_Server_GWY, TCP_SERVER_LOCAL_GWY);
		strcpy(wf._TCP_Local_Server_MSK, TCP_SERVER_LOCAL_MSK);
		strcpy(wf._TCP_Local_Server_Port, TCP_PORT_LOCAL);
		wf._TCP_Local_Server_EN=0;							//Habilito el Servidor Local
		wf._estado_conexion=100;//Si no se define no arranca	//wf._estado_conexion=1;					//Arranco en WiFi Desconectado
		wf._automatizacion=WF_CONNECT_TCP;//wf._automatizacion=WF_SEND;
		wf._NO_IP=1;
		wf._DBG_EN=1;

		//-----------------------1 Lectura de UID ------------------------//
		/* El Nro de UID se compone de 96 bits
		 * UID[0]=Coordenadas X e Y en el wafer
		 * UID[1]=Lote parcial + Nro de wafer
		 * UID[1]=Nro de Lote
		 * Por probabilidad vamos a tomar como UID a visualizar como AP los primeros 32 bits
		 */
			UID[0] = *(__IO uint32_t *)(UID_BASE_ADDRESS);
			UID[1] = *(__IO uint32_t *)(UID_BASE_ADDRESS + 4U);
			UID[2] = *(__IO uint32_t *)(UID_BASE_ADDRESS + 8U);
			INTOA(UID[0],dummyArray);
			strcpy(APWFUID,dummyArray);
			dummyArray[0]='-';
			dummyArray[1]='\0';
			strncat(APWFUID,dummyArray,strlen(dummyArray));
			dummyArray[0]='\0';
			INTOA(UID[1],dummyArray);
			strcpy(wf._WF_AP_Pass,dummyArray);
			strncat(APWFUID,dummyArray,strlen(dummyArray));
			strcpy(wf._WF_AP_SSID,"RIOT-");
			strncat(wf._WF_AP_SSID,APWFUID,strlen(APWFUID));

		//-----------------------0 Lectura de UID ------------------------//
		//----------------------1 ModBUS -----------------------//

			ModBUS_Config(&mb_lr);		//ETHERNET como cliente TCP envía  ModBUS
			mb_lr._mode = CLIENTE;
			ModBUS_Config(&mb_eth);		//ETHERNET como cliente TCP envía  ModBUS
			mb_eth._mode = CLIENTE;

		//----------------------0 ModBUS -----------------------//
		//-----------------------1 ETHERNET W5100 Environment-------------------------//

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
		//	S0 Client IP ADDRESS
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

		//-----------------------0 ETHERNET W5100 Environment-------------------------//
		// -----------1 Seteo de módulo Ethernet W5100 ----------- //
			spi_no_debug=1;
			ETH.NSS_PORT=GPIOB;
			ETH.NSS_PIN=GPIO_PIN_6;
			ETH.SPI= &hspi3;
		// -----------0 Seteo de módulo Ethernet W5100 ----------- //


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


  //------------------1 Habilitacion de dispositivos ---------------//

    HAL_GPIO_WritePin(GPIOB, LR_RST_Pin, GPIO_PIN_SET);		//Habilito LoRa
    HAL_GPIO_WritePin(GPIOA, MBUS_CTRL_Pin, GPIO_PIN_RESET);	//Habilito 485 para RX
    HAL_GPIO_WritePin(GPIOA, WF_EN_RST_Pin, GPIO_PIN_SET);	//Habilito WiFi
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);				//Habilito ETHERNET

  //------------------0 Habilitacion de dispositivos ---------------//
  //------------------1 Se visualiza SSID y AP ---------------//
	ITM0_Write("\r\n\r\n\r\n SYS-uC ID :",strlen("\r\n\r\n\r\n SYS-uC ID :"));
	ITM0_Write(APWFUID,strlen(APWFUID));
	ITM0_Write( "\r\n\r\n\r\n SYS-AP-SSID :", strlen("\r\n\r\n\r\n SYS-AP-SSID :"));
	ITM0_Write( wf._WF_AP_SSID, strlen(wf._WF_AP_SSID));
	ITM0_Write( "\r\n\r\n\r\n SYS-AP-PASS : ", strlen("\r\n\r\n\r\n SYS-AP-PASS : "));
	ITM0_Write( wf._WF_AP_Pass, strlen(wf._WF_AP_Pass));
	ITM0_Write("\r\n\r\n\r\n SYS-ARRANQUE",strlen("\r\n\r\n\r\n SYS-ARRANQUE"));
	if(SYS_DEBUG_EN==1)
		{
			HAL_UART_Transmit(&huart6, "\r\n\r\n\r\n SYS-AP-SSID :", strlen("\r\n\r\n\r\n SYS-AP-SSID :"), 100);
			HAL_UART_Transmit(&huart6, wf._WF_AP_SSID, strlen(wf._WF_AP_SSID), 100);
			HAL_UART_Transmit(&huart6, "\r\n\r\n\r\n SYS-AP-PASS : ", strlen("\r\n\r\n\r\n SYS-AP-PASS : "), 100);
			HAL_UART_Transmit(&huart6, wf._WF_AP_Pass, strlen(wf._WF_AP_Pass), 100);
			HAL_UART_Transmit(&huart6, "\r\n SYS-ARRANQUE", strlen("\r\n SYS-ARRANQUE"), 100);
		}
	//------------------0 Se visualiza SSID y AP ---------------//

  //Carga de valores en la estructura

  /*strcpy(NVS._WIFI_IP,WIFI__IP);
  strcpy(NVS._WIFI_MASK,WIFI_MASK);
  strcpy(NVS._WIFI_PORT,EP_PORT);
  strcpy(NVS._ETH_PORT,ETHERNET_PORT);
  strcpy(NVS._ETH_IP,ETHERNET_IP);
  strcpy(NVS._ETH_TRGT_IP,ETHERNET_TRGT_IP);
  strcpy(NVS._ETH_MASK,ETHERNET_MASK);
  strcpy(NVS._SERVER,EP_SERVER);
  strcpy(NVS._LORA_ADDR,LORA_ADDR);
  strcpy(NVS._LORA_NET_ID,LORA_NET_ID);
  strcpy(NVS._LORA_NCPIN,LORA_NCPIN);
  strcpy(NVS._LORA_BAND,LORA_BAND);
  strcpy(NVS._MBUS_REG,MBUS_REG);
  strcpy(NVS._MBUS_ID,MBUS_ID);
  strcpy(NVS._MBUS_CODE,MBUS_CODE);
  strcpy(NVS._MBUS_SRVR,MBUS_SRVR);
  strcpy(NVS._WIFI_PASS,WIFI_PASS);
  strcpy(NVS._WIFI_SSID,WIFI_NET);*/
  //------------------1 Revisión de datos guardados en memoria ---------------//
  HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1);

  if(BKP_RG_BYTE(&hrtc,READ,LORA,BAND, READ_FUNCTION_8)==0)
  {

	  createAccessPoint(&wf,&huart1);
	  SYS_configData=1;
	  while(SYS_configData==1)
	  {
		  if(FLAG_UART1_WF==1)
		  {

			dummy2=strlen(":GET /192.168.4.1:80?A=");
			if(FT_String_ND(UART_RX_vect_hld,&UART_RX_items,":GET /192.168.4.1:80?A=",&dummy2,wf._uartRCVD_tok,wf._n_tok,&dummy,wf._id_conn,wf._overflowVector,FIND)==1)
				{

					//if(leerValoresAP()==1) SYS_configData=0; //Salgo si leí valore y grabé

					BKP_AP_EXTRACT(&NVS,UART_RX_vect_hld,UART_RX_items);
					ITM0_Write("\r\n SYS-Escritura de valores en registros de back up",strlen("\r\n SYS-Escritura de valores en registros de back up"));
					if(SYS_DEBUG_EN==1) HAL_UART_Transmit(&huart6, "\r\n SYS-Escritura de valores en registros de back up", strlen("\r\n SYS-Escritura de valores en registros de back up"), 100);
					BKP_REG_RW(&hrtc, WRITE, &NVS);
					BKP_REG_SHW(&NVS,&huart6,SYS_DEBUG_EN);

					if(SYS_DEBUG_EN==1)
						{
						HAL_UART_Transmit(&huart6, "\r\n SYS-Datos recibidos de AP", strlen("\r\n SYS-Datos recibidos de AP"), 100);
					    HAL_UART_Transmit(&huart1, "AT+CIPSEND=0,66\r\n", strlen("AT+CIPSEND=0,66\r\n"), 100);
					    HAL_Delay(500);
					    HAL_UART_Transmit(&huart1, WEB, strlen(WEB), 100);
						HAL_Delay(100);
						HAL_UART_Transmit(&huart1, "AT+CIPCLOSE=0\r\n", strlen("AT+CIPCLOSE=0\r\n"), 100);
						}
					SYS_configData=0;
					SYS_SPI_ETH_READ_EN=1;
					FLAG_UART1_WF=0;
				}
				else{
					  dummy2=strlen("+IPD,0,");
					  if(FT_String_ND(UART_RX_vect_hld,&UART_RX_items,"+IPD,0,",&dummy2,wf._uartRCVD_tok,wf._n_tok,&dummy,wf._id_conn,wf._overflowVector,FIND)==1)
					  {
						  //------------------------ PAGINA WEB ------------------------ //
						  HAL_UART_Transmit(&huart1, "AT+CIPSEND=0,925\r\n", strlen("AT+CIPSEND=0,925\r\n"), 100);
						  HAL_Delay(500);
						  HAL_UART_Transmit(&huart1, WEB3, strlen(WEB3), 100);
						  HAL_Delay(500);
						  HAL_UART_Transmit(&huart1, "AT+CIPSEND=0,1016\r\n", strlen("AT+CIPSEND=0,1015\r\n"), 100);
						  HAL_Delay(500);
						  HAL_UART_Transmit(&huart1, WEB4, strlen(WEB4), 100);
						  HAL_Delay(100);
						  HAL_UART_Transmit(&huart1, "AT+CIPCLOSE=0\r\n", strlen("AT+CIPCLOSE=0\r\n"), 100);
						  HAL_Delay(10);
						  //------------------------ PAGINA WEB ------------------------ //
						  FLAG_UART1_WF=0;
						  }else
							  {
							  FLAG_UART1_WF=0;
							  }
					}
		  }
	  }
  }
  else{
		  ITM0_Write("\r\n SYS-Lectura de valores en registros de back up",strlen("\r\n SYS-Lectura de valores en registros de back up"));
		  if(SYS_DEBUG_EN==1) HAL_UART_Transmit(&huart6, "\r\n SYS-Lectura de valores en registros de back up", strlen("\r\n SYS-Lectura de valores en registros de back up"), 100);
		  BKP_REG_RW(&hrtc, READ, &NVS);
		  BKP_REG_SHW(&NVS,&huart6,SYS_DEBUG_EN);
  	  }
  //------------------0 Revisión de datos guardados en memoria ---------------//

  HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1);
  HAL_UART_Receive_IT(&huart2,(uint8_t *)UART2_RX_byte,1);
  HAL_UART_Receive_IT(&huart6,(uint8_t *)UART6_RX_byte,1);
  ITM0_Write("\r\nPuerto serie en escucha",strlen("\r\nPuerto serie en escucha"));
  if(SYS_DEBUG_EN==1) HAL_UART_Transmit(&huart6, "\r\nPuerto serie en escucha", strlen("\r\nPuerto serie en escucha"), 100);

  if(SYS_ETH_DBG_EN==1) ITM0_Write("\r\n SET-UP W5100 \r\n",strlen("\r\n SET-UP W5100 \r\n"));

	eth_init(&ETH);
	eth_socket_init(&ETH,0);
	SYS_SPI_ETH_READ_EN=1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(FLAG_UART1_WF==1)
	  {
		  ITM0_Write("\r\nRX UART1",strlen("\r\nRX UART1"));

		  /*dummy2=strlen("+IPD,0,");
		  if(FT_String_ND(UART_RX_vect_hld,&UART_RX_items,"+IPD,0,",&dummy2,wf._uartRCVD_tok,wf._n_tok,&dummy,wf._id_conn,wf._overflowVector,FIND)==1)
		  {
			  //------------------------ PAGINA WEB ------------------------ //
			  HAL_UART_Transmit(&huart1, "AT+CIPSEND=0,925\r\n", strlen("AT+CIPSEND=0,925\r\n"), 100);
			  HAL_Delay(1000);
			  HAL_UART_Transmit(&huart1, WEB3, strlen(WEB3), 100);
			  HAL_Delay(1000);
			  HAL_UART_Transmit(&huart1, "AT+CIPSEND=0,1016\r\n", strlen("AT+CIPSEND=0,1015\r\n"), 100);
			  HAL_Delay(1000);
			  HAL_UART_Transmit(&huart1, WEB4, strlen(WEB4), 100);
			  HAL_Delay(1000);
			  HAL_UART_Transmit(&huart1, "AT+CIPCLOSE=0\r\n", strlen("AT+CIPCLOSE=0\r\n"), 100);
			  HAL_Delay(10);
			  //------------------------ PAGINA WEB ------------------------ //
		  }*/
		  FLAG_UART1_WF=0;
	  }

	  if(FLAG_UART2_485==1)
	  {
		  ITM0_Write("\r\nRX UART2",strlen("\r\nRX UART2"));
		  if(FT_String_ND(UART2_RX_vect_hld,&UART2_RX_items,"PRUEBA DE RECEPCION VIA 485",&dummy3,wf._uartRCVD_tok,wf._n_tok,&dummy,wf._id_conn,wf._overflowVector,FIND)==1)
		  {
			  HAL_GPIO_WritePin(GPIOA, MBUS_CTRL_Pin, GPIO_PIN_SET);	//Habilito 485 para TX
			  HAL_UART_Transmit(&huart2,"TX VIA RS-485\r\n", 15, 100);
			  HAL_GPIO_WritePin(GPIOA, MBUS_CTRL_Pin, GPIO_PIN_RESET);	//Habilito 485 para RX
		  }
		  FLAG_UART2_485=0;
	  }

	  if(FLAG_UART6==1)
	  {
		  ITM0_Write("\r\nRX UART6",strlen("\r\nRX UART6"));
		  if(FT_String_ND(UART6_RX_vect_hld,&UART6_RX_items,"+RCV",&dummy4,wf._uartRCVD_tok,wf._n_tok,&dummy,wf._id_conn,wf._overflowVector,FIND)==1)
		  {
			  HAL_UART_Transmit(&huart6,"AT\r\n", 4, 100);
		  }
		  FLAG_UART6=0;
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

  /*Configure GPIO pins : Q0_1_Pin WF_EN_RST_Pin DBG_PIN_Pin */
  GPIO_InitStruct.Pin = Q0_1_Pin|WF_EN_RST_Pin|DBG_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MBUS_CTRL_Pin */
  GPIO_InitStruct.Pin = MBUS_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MBUS_CTRL_GPIO_Port, &GPIO_InitStruct);

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
   ms_ticks++;

	if (mseg==500)
		{
		   HAL_GPIO_TogglePin(GPIOB, FALLA_Pin);
		   HAL_GPIO_TogglePin(GPIOB, CNN_Pin);
		   HAL_GPIO_TogglePin(GPIOB, ALIM_Pin);
		   mseg=0;
		}

	if(mb_eth._w_answer) MB_TOUT_ticks++;
	if ( mb_eth._w_answer && (mb_eth._timeout < MB_TOUT_ticks))
		{
			mb_eth._w_answer=0;
			MB_TOUT_ticks=0;
		}

   if (ms_ticks==300)
     {
   	ms_ticks=0;
   	min_ticks++;

   	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

   	if(spi_no_debug)
   	  {
   	  if(SYS_SPI_ETH_READ_EN==1)
   	  {
   	     ETH.S0_status=eth_rd_SOCKET_STAT(&ETH,0);

   		  switch(ETH.S0_status)	//Check Socket status
   	     {
   			 case SOCK_CLOSED :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_CLOSED \r\n",strlen("\r\nS0_SOCK_CLOSED \r\n"));}
   					 eth_wr_SOCKET_CMD(&ETH, 0 ,OPEN );
   					 // Si no tengo intento de ARP por 5 segundos vuelvo a inicializar
   					 if(ETH.ETH_WDG>=5000)
   					 {
   						 ETH.ETH_WDG=0;
   						 eth_init(&ETH);
   						 eth_socket_init(&ETH,0);
   					 }

   				 }
   			 break;
   			 case  SOCK_INIT :
   				 {
   					 if(ETH.S0_ENserver == 1)
   					 {
   						 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_INIT \r\n",strlen("\r\nS0_SOCK_INIT \r\n"));}
   							eth_wr_SOCKET_CMD(&ETH, 0, LISTEN );
   							ETH.ETH_WDG=0;
   					 }
   					 else
   					 {
   						 	eth_wr_SOCKET_CMD(&ETH,0, CONNECT);																				//only for server
   						 	if (SYS_ETH_DBG_EN == 1)
   						 	{
   						 		ITM0_Write("\r\nETH-W5100-CONNECT\r\n",strlen("\r\nETH-W5100-CONNECT\r\n"));
   						 	}
   						 	ETH.ETH_WDG=0;
   					 }

   				 }
   			 break;
   			 case SOCK_LISTEN :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_LISTEN \r\n",strlen("\r\nS0_SOCK_LISTEN \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_SYNSENT :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_SYNSENT \r\n",strlen("\r\nS0_SOCK_SYNSENT \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_SYNRECV :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_SYNRECV \r\n",strlen("\r\nS0_SOCK_SYNRECV \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_ESTABLISHED :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_ESTABLISHED \r\n",strlen("\r\nS0_SOCK_ESTABLISHED \r\n"));}
   					 ETH.ETH_WDG=0;

   					if (ETH.S0_ENserver == 1)  // Si el puerto Ethernet actúa como server (Recibe datos conexión mas pedido mbus
   					{

   							S0_get_size = SPI_ETH_REG(&ETH, S0_RX_SZ_ADDR_BASEHH,S0_RX_SZ_ADDR_BASEHL ,SPI_READ, spi_Data,2);
   							if(S0_get_size != 0x00)
   							{
   								eth_rd_SOCKET_DATA(&ETH,0,&rx_mem_pointer,S0_get_size); // read socket data
   								SPI_ETH_WR_REG_16(&ETH,S0_RX_RD0,rx_mem_pointer );		// write rx memory pointer
   								eth_wr_SOCKET_CMD(&ETH,0,RECV);							// write command to execute
   								while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
   								{}

   								CopiaVector(mb_eth._MBUS_RCVD, ETH.data, S0_get_size, 0, 0 );
   								mb_eth._n_MBUS_RCVD=S0_get_size;

   								//if(S0_get_size > 0)	{ ETH.S0_data_available=1;}					//Flag data received

   								if(ModBUS_Check(mb_eth._MBUS_RCVD, mb_eth._n_MBUS_RCVD))		//Ckecks ModBUS type data
   								{
   									ModBUS(&mb_eth);										//ModBUS protocol execution
   									CopiaVector(ETH.data, mb_eth._MBUS_2SND, mb_eth._n_MBUS_2SND, 0, 0);
   								}
   								else
   								{
   									if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\n NO MBUS \r\n",strlen("\r\n\r\n NO MBUS \r\n\r\n"));}
   								}

   								send_size=mb_eth._n_MBUS_2SND;  //ModBUS data qty

   								eth_wr_SOCKET_DATA(&ETH,0, &tx_mem_pointer, send_size);	// write socket data
   								SPI_ETH_WR_REG_16(&ETH,0x424,tx_mem_pointer);			// write tx memory pointer
   								eth_wr_SOCKET_CMD(&ETH,0,SEND);							// write command to execute
   								while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
   								{}

   							}
   					}
   					else	// Puerto ethernet labura como esclavo, se conecta al server para pedir datos
   					{

   						if (mb_eth._w_answer==0)
   						{
   							//Si ya envié vuelvo a enviar

   							ETH.data[0]=0x00;
   							ETH.data[1]=0x00;
   							ETH.data[2]=0x00;
   							ETH.data[3]=0x00;
   							ETH.data[4]=0x00;
   							ETH.data[5]=0x06;
   							ETH.data[6]=0x01;
   							ETH.data[7]=0x03;
   							ETH.data[8]=0x00;
   							ETH.data[9]=0x00;
   							ETH.data[10]=0x00;
   							ETH.data[11]=0x0A;
   							send_size=12;

   							ModBUS_F03_Request(&mb_eth,0,15);
   							CopiaVector(ETH.data, mb_eth._MBUS_2SND, 12, 0, 0 );

   							eth_wr_SOCKET_DATA(&ETH,0, &tx_mem_pointer, send_size);	// write socket data
   							SPI_ETH_WR_REG_16(&ETH,0x424,tx_mem_pointer);			// write tx memory pointer
   							eth_wr_SOCKET_CMD(&ETH,0,SEND);							// write command to execute
   							while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
   							{}
   							mb_eth._w_answer=1;	// Waiting answer flag
   							MB_TOUT_ticks=0;	// restart counting
   							if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\n SENT MBUS REQ \r\n",strlen("\r\n\r\n SENT MBUS REQ \r\n\r\n"));}
   						}
   						else
   						{
   						S0_get_size = SPI_ETH_REG(&ETH, S0_RX_SZ_ADDR_BASEHH,S0_RX_SZ_ADDR_BASEHL ,SPI_READ, spi_Data,2);
   							if(S0_get_size != 0x00)
   							{
   								eth_rd_SOCKET_DATA(&ETH,0,&rx_mem_pointer,S0_get_size); // read socket data
   								SPI_ETH_WR_REG_16(&ETH,S0_RX_RD0,rx_mem_pointer );		// write rx memory pointer
   								eth_wr_SOCKET_CMD(&ETH,0,RECV);							// write command to execute
   								while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
   								{}

   								CopiaVector(mb_eth._MBUS_RCVD, ETH.data, S0_get_size, 0, 0 );
   								mb_eth._n_MBUS_RCVD=S0_get_size;

   								//if(S0_get_size > 0)	{ ETH.S0_data_available=1;}

   								if(ModBUS_Check(mb_eth._MBUS_RCVD, mb_eth._n_MBUS_RCVD))		//Ckecks ModBUS type data
   									{
   										mb_eth._w_answer=0;  									//Si el mensaje recibido ya es modbus digo que ya recibi
   										MB_TOUT_ticks=0;
   										ModBUS(&mb_eth);										//ModBUS protocol execution
   										CopiaVector(ETH.swap, mb_eth._MBUS_RCVD, mb_eth._n_MBUS_RCVD, 0, 0);
   										CopiaVector(mb_wf._Holding_Registers, mb_eth._Holding_Registers, 64, 0, 0);
   										ETH.S0_data_available=1;	//Informa que se ha recibido un dato y es ModBUS
   										if (SYS_ETH_DBG_EN == 1)
   										{
   											HAL_UART_Transmit_IT(&huart2,"\r\nMBUS RCVD\r\n",strlen("\r\nMBUS RCVD\r\n"));
   											ITM0_Write("\r\n RCVD MBUS REQ \r\n",strlen("\r\n\r\n RCVD MBUS REQ \r\n\r\n"));
   										}
   									}
   									else
   										{
   										if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\n NO MBUS \r\n",strlen("\r\n\r\n NO MBUS \r\n\r\n"));}
   										}


   							}
   						}
   					}
   				 }
   			 break;
   			 case SOCK_FIN_WAIT :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_FIN_WAIT \r\n",strlen("\r\nS0_SOCK_FIN_WAIT \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_CLOSING :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_CLOSING \r\n",strlen("\r\nS0_SOCK_CLOSING \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case  SOCK_TIME_WAIT :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_TIME_WAIT \r\n",strlen("\r\nS0_SOCK_TIME_WAIT \r\n"));}
   					eth_wr_SOCKET_CMD(&ETH,0, DISCON );
   					while( SPI_ETH_REG(&ETH, S0_CR_ADDR_BASEH,S0_CR_ADDR_BASEL ,SPI_READ, spi_Data,1))
   					{}
   					ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_CLOSE_WAIT :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_CLOSE_WAIT \r\n",strlen("\r\nS0_SOCK_CLOSE_WAIT \r\n"));}
   					eth_wr_SOCKET_CMD(&ETH,0,DISCON );
   					while( SPI_ETH_REG(&ETH, S0_CR_ADDR_BASEH,S0_CR_ADDR_BASEL ,SPI_READ, spi_Data,1))
   					{}
   					ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_LAST_ACK :
   				 {
   					 if (SYS_ETH_DBG_EN == 1)
   					 {
   						 ITM0_Write("\r\nS0_SOCK_LAST_ACK \r\n",strlen("\r\nS0_SOCK_LAST_ACK \r\n"));
   					 }
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_UDP :
   				 {
   					 if (SYS_ETH_DBG_EN == 1){ ITM0_Write("\r\nS0_SOCK_UDP \r\n",strlen("\r\nS0_SOCK_UDP \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case  SOCK_IPRAW :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_IPRAW \r\n",strlen("\r\nS0_SOCK_IPRAW \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case  SOCK_MACRAW :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_MACRAW \r\n",strlen("\r\nS0_SOCK_MACRAW \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;
   			 case SOCK_PPOE :
   				 {
   					 if (SYS_ETH_DBG_EN == 1) {ITM0_Write("\r\nS0_SOCK_PPOE \r\n",strlen("\r\nS0_SOCK_PPOE \r\n"));}
   					 ETH.ETH_WDG=0;
   				 }
   			 break;

   			 default:
   				 {

   				 }
   	     }
   	  }
   	  }else
   	  	  {
   		  SPI_ETH(&ETH);
   	  	  }
   	  if(min_ticks==2)//if(min_ticks==10)
   		  {
   		  	  min_ticks=0;  /* SETEO CADA 2 min*/
   		  }
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
			//if(UART_RX_pos>=1022) UART_RX_pos=1022;
			if(UART_RX_pos>=4095) UART_RX_pos=4095;
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
			//if(UART6_RX_pos>=512) UART6_RX_pos=512;
			if(UART6_RX_pos>=4095) UART6_RX_pos=4095;
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
				 FLAG_UART1_WF=1;
				 EN_UART1_TMR=0;
				 UART_RX_items=UART_RX_pos;
				 UART_RX_pos=0;
				 UART_RX_vect[4095]='\0';//UART_RX_vect[1022]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(UART_RX_vect_hld,UART_RX_vect,UART_RX_items,1,CMP_VECT);
				 // Re-envío de info al ESP//
				 HAL_UART_Transmit_IT(&huart6, UART_RX_vect_hld, UART_RX_items);
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
				 FLAG_UART2_485=1;
				 EN_UART2_TMR=0;
				 UART2_RX_items=UART2_RX_pos;
				 UART2_RX_pos=0;
				 UART2_RX_vect[512]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(UART2_RX_vect_hld,UART2_RX_vect,UART2_RX_items,1,CMP_VECT);
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
				 UART6_RX_vect[4095]='\0'; //UART6_RX_vect[512]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(UART6_RX_vect_hld,UART6_RX_vect,UART6_RX_items,1,CMP_VECT);
				 // Re-envío de info al ESP//
				 HAL_UART_Transmit_IT(&huart1, UART6_RX_vect_hld, UART6_RX_items);
				 HAL_UART_Receive_IT(&huart6,(uint8_t *)UART6_RX_byte,1); //Habilito le recepcón de puerto serie al terminar
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
