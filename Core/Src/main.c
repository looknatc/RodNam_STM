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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ESP8266_HAL.h"
//#include "DHT.h"
//#include "MYDHT22.h"
#include <stdarg.h>
//#include "DHT22.h"

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t i2c2add = 0x48 << 1; // Use 8-bit address
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t Rx_byte;
volatile uint8_t Rx_data[10240];
volatile uint8_t Rx_indx = 0;

volatile uint8_t Rx_byte2;
volatile uint8_t Rx_data2[10240];
volatile uint8_t Rx_indx2 = 0;

char uart_recv;
char str[10240];
char tmp[1000];
int counter = 0;
float parTemperature = 36;
float parSoilHumidity = 70;
int parHumidity = 50;
int parWater = 5;
int parDoEvery = 20;

void serialSend(char* str){
	HAL_UART_Transmit(&huart2, str, strlen(str), 10);
}

#define log(x)	serialSend(x)
void wifisend(char* str){
	HAL_UART_Transmit(&huart1, str, strlen(str), 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart1) { // Current UART
		Rx_data[Rx_indx++] = Rx_byte;    // Add data to Rx_Buffer
		HAL_UART_Receive_IT(&huart1, &Rx_byte, 1);
	}
	if (huart == &huart2) { // Current UART
		Rx_data2[Rx_indx2++] = Rx_byte2;    // Add data to Rx_Buffer
		HAL_UART_Receive_IT(&huart2, &Rx_byte2, 1);
	}
}

void sendCommand(char* id,float hum,int soil,float temp,float light,int relay){
	char command[128];
	sprintf(command,"sendData('%s %f %d %f %f %d')\r\n",id,hum,soil,temp,light,relay);
	log(command);
	wifisend(command);
}

int  readLineUart(char* buffer){
	for(int i = 0;i< Rx_indx;i++){
		buffer[i] = Rx_data[i];
	}
	int ret = Rx_indx;
	buffer[Rx_indx] = 0;
	Rx_indx = 0;
	return ret;
}

int  readLineUart2(char* buffer){
	for(int i = 0;i< Rx_indx2;i++){
		buffer[i] = Rx_data2[i];
	}
	int ret = Rx_indx2;
	buffer[Rx_indx2] = 0;
	Rx_indx2 = 0;
	return ret;
}

//char temp[1024];
//char humid[1024];
//char soil[1024];

char *deviceID = "Naotest8";

//DHT_DataTypedef DHT11_Data;
float Temperature, Humidity = 1.0;
uint16_t SoilHumidity = 0;
float light = 40.0;
int relayRunning = 0;
void powerOnSensor(){
	log("powerOnSensor\n");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1); //On
	// delay 2 sec for DHT
	HAL_Delay(2000);
}
void powerOffSensor(){
	log("powerOffSensor\n");
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0); //On

}
void relayOn(){
	log("relayOn\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); //On
	relayRunning = 1;
}

void relayOff(){
	log("relayOff\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); //On
	relayRunning = 0;
}
void dumpData(char *buffer,int len){
	for(int i=0;i<len;i++){
		sprintf(tmp,"%02X ",buffer[i]);log(tmp);
	}
	log("\r\n");
}
int getWifiStatus(){
	char cmd[64];
	//wifisend("showConnectionStatus()\n");
	wifisend("print(connected)\n");
	HAL_Delay(200);
	int numread = readLineUart(str);

	if(numread >0){
		char *ptr = strstr(str,"\n");
		if(ptr!=NULL){
			//log(ptr);
			if(sscanf(ptr,"%s",cmd) == 1){
				//sprintf(tmp,"%d %d\r\n",numread,strlen(str));log(tmp);
				//sprintf(tmp,"[%s]",ptr);log(tmp);
				//dumpData(ptr,strlen(ptr));
				if(strcmp(cmd,"true")==0){
					log("connected\r\n");
					return 1;
				}
				else{
					log("disconnect\r\n");
					return 0;
				}
			}
		}
	}
}
int getDHT(int debug){
	if(debug == 2){
		powerOnSensor();
	}

	char tmp[64];
	wifisend("getDHT()\n");
	HAL_Delay(2000);
	int numread = readLineUart(str);
	//sprintf(tmplog,"numread=%d\r\n",numread);log(tmplog);
	if(numread >0){
		if(debug == 1){
			log(str);
		}
		char *ptr = strstr(str,"\n");
		if(ptr!=NULL){
			//log(ptr);
			if(sscanf(ptr,"%s %f %f",&tmp[0],&Temperature,&Humidity)==3){
				if(debug == 2){
					powerOffSensor();
				}

				return 1;
			}
		}
	}
	if(debug == 2){
		powerOffSensor();
	}
	return 0;
}

void getSoil(int debug){
	if(debug == 2){
		powerOnSensor();
	}
	HAL_ADC_Start(&hadc1);
	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 100);
	// Read The ADC Conversion Result & Map It To PWM DutyCycle
	SoilHumidity = HAL_ADC_GetValue(&hadc1);
	if(debug == 2){
		powerOffSensor();
	}
}

void upload(){
	sendCommand(deviceID, Humidity, SoilHumidity, Temperature, light, relayRunning);
	HAL_Delay(2000);
	int numread = readLineUart(str);
	log(str);
}

int shoudWater(float Humidity, int SoilHumidity, float Temperature){
//	temp //อุณหภูมิ
//	x //ค่าความชื้นในดิน
//	if (SoilHumidity <1400) SoilHumidity =1400;
//	if (SoilHumidity >3600) SoilHumidity =3600;
//	loadSetting();
	float moisture = (4500.0 - SoilHumidity)/4000.0*100.0; //ความชื้น
	if (moisture < parSoilHumidity ) {
		sprintf(tmp,"bec soil = %f < %f \r\n",moisture,parSoilHumidity );
		log(tmp);
		return 1;

	}
	else if(Temperature > parTemperature){
		sprintf(tmp,"bec temp = %f > %f \r\n",Temperature,parTemperature );
		log(tmp);
		return 1;
	}
	sprintf(tmp,"DONT bec soil = %f % [%d] temp = %f \r\n",moisture,SoilHumidity,Temperature );
	log(tmp);

//	if(rand()%5 == 0){
//		return 1;
//	}
	return 0;
}

void readSensor()
{
	powerOnSensor();
	getDHT(0);
	getSoil(0);
	powerOffSensor();
}

void showESPOutput()
{
	int numread = readLineUart(str);
	if(numread>0){
		log(str);
	}
}
void getSetting()
{
	int numread = readLineUart(str);
	if(numread>0){
//		log(str);
		char *ptr = strstr(str,"\n");
//		float parTemperature = 36.0;
//		float parSoilHumidity = 70;
//		float parHumidity = 50;
//		float parWater = 5;
//		float parDoEvery = 20;
		for(int i = 0;i< strlen(ptr);i++){
			if(ptr[i] == '"'){
				ptr[i] = ' ';
			}
		}
		log(ptr);
//		if(sscanf(str,"%d %s",&tmp[0],&parTemperature,&parSoilHumidity,&parHumidity,&parWater,&parDoEvery) ==6){
//					log("good\r\n");
//				}
		int t;
		if(sscanf(ptr,"%s %d %f %f %d %d %d",&tmp[0],&t,&parTemperature,&parSoilHumidity,&parHumidity,&parWater,&parDoEvery) ==7){
			log("good\r\n");
		}
		sprintf(tmp,"setting in STM32: %f %f %d %d %d \r\n",parTemperature,parSoilHumidity,parHumidity,parWater,parDoEvery);
		log(tmp);
	}
}
void loadSetting()
{
	wifisend("loadSetting()\n");
	HAL_Delay(1000);
//	showESPOutput();
	getSetting();

}

void processCommand()
{
	char cmd[1024];
	int numread = readLineUart2(cmd);
	if(numread>0){
		sprintf(tmp,"cmd=%s",cmd);log(tmp);
		if(strcmp(cmd,"water\n")==0){
			loadSetting();
			readSensor();
			relayOn();
			relayRunning = 2; 	// make it as manual watering
			upload();
			HAL_Delay(1000*parWater);
			relayOff();
			return;
		}
		if(strcmp(cmd,"upload\n")==0){
			readSensor();
			upload();
			return;
		}
		if(strcmp(cmd,"status\n")==0){
			readSensor();
			sprintf(tmp,"%s %f %d %f %f %d\r\n",deviceID, Humidity, SoilHumidity, Temperature, light, relayRunning);
			log(tmp);
			return;
		}
		if(strcmp(cmd,"wifi\n")==0){
			getWifiStatus();
			return;
		}
		if(strcmp(cmd,"ls\n")==0){
			loadSetting();
			return;
		}
		if(cmd[0] == '#'){
			wifisend(&cmd[1]);
			HAL_Delay(2000);
			showESPOutput();
			return;
		}
		sprintf(tmp,"Unknow command: %s",cmd);log(tmp);
		return;
	}
}

void waitingForESP()
{
	// delay 5 sec for ESP reboot
	HAL_Delay(5000);
	log("waitingForESP start\r\n");
	for(int i=0;i<60;i++){
		if(getWifiStatus()==1){
			log("ESP WIFI ready\r\n");
			break;
		}else{
			sprintf(tmp,"wait for ESP WIFI %d\r\n",i+1);log(tmp);
			HAL_Delay(1000);
		}
	}
	log("waitingForESP end\r\n");
}
//extern void delay(volatile uint32_t microseconds);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_ADCEx_Calibration_Start(&hadc1);
//  ESP_Init("muminoiais_5G", "99775533");
//  HAL_UART_Receive_IT(&huart1, &uart_recv, 1);
  HAL_UART_Receive_IT(&huart1, &Rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &Rx_byte2, 1);
  log("**-----------------------------------------------\r\n");
  char UART1_rxBuffer[100] = {'a'};





  /*constrain part */
  float tempConst = 30.0;
  float soilConst = 1000; // low = moist
  float  humidConst = 60;  // high = moist
  int count = 1;

  //waiting for ESP ready by checking wifistatus
  waitingForESP();
  loadSetting();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	count++;
	sprintf(tmp,"count=%d\r\n",count);log(tmp);
//	loadSetting();
	if(count % (60*parDoEvery) == 0){
		loadSetting();
	//if(count % (10) == 0){
		powerOnSensor();
		if(getDHT(0)){
			getSoil(0);
//			readSensor();
			sprintf(tmp,"%s %f %d %f %f %d\r\n",deviceID, Humidity, SoilHumidity, Temperature, light, relayRunning);
			//sprintf(tmp,"%s %.2f %.2f %.2f %.2f %d\r\n",deviceID, Humidity, SoilHumidity, Temperature, light, relayRunning);
			log(tmp);
			if(shoudWater(Humidity, SoilHumidity, Temperature)){
				relayOn();
				upload();
				HAL_Delay(1000*parWater);
				relayOff();
			}else{
				upload();
			}
		}
		powerOffSensor();
	}
	HAL_Delay(1000);
	showESPOutput();
	processCommand();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_7;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

