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
#include "ESP8266_HAL.h"
#include "DHT.h"
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
int counter = 0;
void log(char* str){
	HAL_UART_Transmit(&huart2, str, strlen(str), 10);
}
void wifisend(char* str){
	HAL_UART_Transmit(&huart1, str, strlen(str), 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart1) { // Current UART
				Rx_data[Rx_indx++] = Rx_byte;    // Add data to Rx_Buffer
				HAL_UART_Receive_IT(&huart1, &Rx_byte, 1);
	}


//	if (huart == &huart2) { // Current UART
//		Rx_data2[Rx_indx2++] = Rx_byte2;    // Add data to Rx_Buffer
//		HAL_UART_Receive_IT(&huart2, &Rx_byte2, 1);
//	}

//	if(huart == &huart1){
//		counter++;
//
//	}
//	log(">>>>>in\r\n");
//	HAL_UART_Receive_IT(&huart1,&uart_recv,1);
//	log("called\r\n");
//	if(huart == &huart1){
//		sprintf(str,"%d\r\n",uart_recv);
//		log(str);
////		if(uart_recv ==1){
////			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
////		}
////		else{
////			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
////		}
//
//	}
//	else{
//		log("missUART\r\n");
//	}

}

void sendCommand(char* id,float hum,int soil,float temp,float light){
	char command[128];
	sprintf(command,"sendData('%s %f %d %f %f')\r\n",id,hum,soil,temp,light);
	log(command);
	HAL_UART_Transmit(&huart1, command, strlen(command), 1000);

	if(readLineUart(str)>0){
		log(str);
		log("\r\n");
	}
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

int checkCommandUart2(){
	if(Rx_indx2 == 2){
		if(Rx_data2[0] == 'a' && Rx_data2[1] == 'a'){
			Rx_indx2 = 0;
			return 1;
		}
	}
	Rx_indx2 = 0;
	return 0;

//	for(int i = 0;i< Rx_indx;i++){
//		buffer[i] = Rx_data[i];
//	}
//	int ret = Rx_indx;
//	buffer[Rx_indx] = 0;
//	Rx_indx = 0;
//	return ret;
}
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
//	log("<receive\n\r");
//	if(hi2c == &hi2c2){
//			if(i2c_recv ==1){
//				log("on\n\r");
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
//			}
//			else{
//				log("off\n\r");
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
//			}
////			HAL_I2C_Slave_Receive_IT(hi2c,&i2c_recv,1);
//		}
//}
//char str[1024];
char temp[1024];
char humid[1024];
char soil[1024];

DHT_DataTypedef DHT11_Data;
float Temperature, Humidity = 1.0;
int relayRunning = 0;
void relayOn(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); //On
	relayRunning = 1;
	//HAL_Delay(5000);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1); //On
}

void relayOff(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); //On
	relayRunning = 2;
}
extern void delay(volatile uint32_t microseconds);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t SoilHumidity = 0;
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
//  HAL_UART_Receive_IT(&huart2, &Rx_byte2, 1);
  log("**-----------------------------------------------\r\n");
  char UART1_rxBuffer[100] = {'a'};
  char* tmplog[1000];
  char xx[128];
  float light = 40.0;
  sprintf(xx,"Naotest8");

  /*constrain part */
  float tempConst = 30.0;
  float soilConst = 1000; // low = moist
  float  humidConst = 60;  // high = moist
  int count = 0;





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_Delay(2000);
  log("Setup Start\r\n");
  //log("relayOn\r\n");
  //relayOn();
  /* DO NOT DELETE SET UP STUPID HAL_DELAY ------------------------------------------------------------------ */
//    DHT_GetData(&DHT11_Data);
  /* DO NOT DELETE SET UP STUPID HAL_DELAY ------------------------------------------------------------------ */

  log("Setup End\r\n");
  while (1)
  {
//	  getTemperatureC();

	  /* relay testing 2 */
	  	count++;
	  	sprintf(tmplog,"#%d\r\n",count);
	  	log(tmplog);
	  	if(count % 2 == 0){
	  		//if(relayRunning == 1){
	  			relayOn();
	  			HAL_Delay(5000);
	  			relayOff();
	  			HAL_Delay(2000);
	  		//}else{

	  		//}
	  	}

	/*relay testing*/


//	  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); //On
//	  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); //Off
//	  	log("on\r\n");
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); //Off
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); //Off
//		log("off\r\n");
//		HAL_Delay(2000);


/*sensor reading*/
//
		HAL_ADC_Start(&hadc1);
		// Poll ADC1 Perihperal & TimeOut = 1mSec
		HAL_ADC_PollForConversion(&hadc1, 1);
		// Read The ADC Conversion Result & Map It To PWM DutyCycle
		SoilHumidity = HAL_ADC_GetValue(&hadc1);
		sprintf(soil, "soil:  %d\r\n", SoilHumidity);
		log(soil);
//		HAL_Delay(6000);

//	  	log("delay\r\n");
//	  	HAL_Delay(1000);
	  	//delay(1000*1000);

	  	log("DHT_GetData start\r\n");
		DHT_GetData(&DHT11_Data);
		log("DHT_GetData end\r\n");
		Temperature = DHT11_Data.Temperature;
		Humidity = DHT11_Data.Humidity;
		sprintf(temp, "temp:  %f\r\n", Temperature);
		log(temp);
		sprintf(humid, "humid:  %f\r\n", Humidity);
		log(humid);
		HAL_Delay(10000);

//		/*LOGIC PART*/
//		if(Temperature > tempConst && SoilHumidity > soilConst){
//			// water
//
//		}



//		Uart_sendstring("send 1 2 3", &huart1);
//		log("send 8 8 8\r\n");
//		wifisend("send 9 9 9\r\n");
//		if(HAL_UART_Receive (&huart1, UART1_rxBuffer, 1, 5000) == HAL_OK){
//			log(UART1_rxBuffer);
//			log(">>>>hahaha\r\n");
//		}
//		else{
////			log("readError\r\n");
//		}


	//thiss



//	  int cmd = checkCommandUart2();
//	  sprintf(tmplog,"%d\r\n",cmd);
//	  log(tmplog);
//	  sendCommand(xx, Humidity, SoilHumidity, Temperature, light);
//	  HAL_Delay(60000);
//	  if(cmd >0){
//		  log("aa  OK \r\n");
//	  }
//		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0){
//			HAL_Delay(10);
//			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) ==0){
////				mode = !mode;
////				HAL_UART_Transmit(&huart1, &mode, 1, 100);
//				log(">>>>>send\r\n");
//				  char command[128];
//				  sprintf(command,"sendData('vinca 26 275 30 40')\r\n");
//					HAL_UART_Transmit(&huart1, command, strlen(command), 1000);
//
//					if(readLineUart(str)>0){
//						log(str);
//						log("\r\n");
//					}
////				sendCommand(xx, Humidity, SoilHumidity, Temperature, light);
//				while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) ==0);
//					HAL_Delay(10);
//			}
//		}



//-------------------work----------------------
//	  char command[128];
//	  sprintf(command,"sendData('NAO 26 275 30 40')\r\n");
//
//	  	HAL_UART_Transmit(&huart1, command, strlen(command), 1000);
//
//		if(readLineUart(str)>0){
//			log(str);
//			log("\r\n");
//		}
//	  	if(HAL_UART_Receive(&huart2, &Rx_byte2, 1,1000) == HAL_OK){
//	  		sprintf(tmplog,"%c\r\n",Rx_byte2);
//	  		log(tmplog);
//	  	}
//		HAL_Delay(6000);
//	  ------------------------------

//	  char command[128];
//	  	sprintf(command,"sendData('%s %f %d %f %f')\r\n",id,hum,soil,temp,light);
//	  	log(command);
//	  	HAL_UART_Transmit(&huart1, command, strlen(command), 1000);




//		char command[128];
//		sprintf(command,"sendData('%s %f %d %f %f')\r\n",xx,Humidity,SoilHumidity,Temperature,light);
//		log(command);
//		HAL_UART_Transmit(&huart1, command, strlen(command), 1000);
//
//		if(readLineUart(str)>0){
//			log("-----1\r\n");
//			log(str);
//			log("\r\n");
//		}



//		-----------------------------esp part
		sendCommand(xx, Humidity, SoilHumidity, Temperature, light);
		if(HAL_UART_Receive(&huart2, &Rx_byte2, 1,1000) == HAL_OK){
			log("-----2\r\n");
			sprintf(tmplog,"%c\r\n",Rx_byte2);
			log(tmplog);
		}
		HAL_Delay(30000);
//		-----------------------------esp part





//		readLineUart(str);
//		sprintf(str,"nao  %d\r\n",counter);
//		log(str);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  Server_Start();

  }
  log("-----------------------------------------------\r\n");
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
  RCC_OscInitStruct.PLL.PLLN = 50;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PC4 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
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

