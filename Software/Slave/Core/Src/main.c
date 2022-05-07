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
#include "can.h"
#include "dma.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
static CAN_TxHeaderTypeDef        TxMessage;    //CAN发送的消息的消息头
static CAN_RxHeaderTypeDef        RxMessage;    //CAN接收的消息的消息头

uint8_t Collection_start;
uint8_t Collection_over;
uint8_t order[9] = {0x09,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};//指令集
uint8_t Send_flag;
uint8_t SendNum;

uint16_t ADC_ConvertedValue[4];//DMA存储数据的容器
uint16_t VoltageS[20][15];//阵列传感器的数据
uint8_t Voltage_data[600];//发送的阵列传感器数据

uint8_t Row_A[15] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
uint8_t Row_B[15] = {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1};
uint8_t Row_C[15] = {0,0,0,0,1,1,1,1,0,0,0,0,1,1,1};
uint8_t Row_D[15] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1};//1~14

uint8_t Lists[20] = {13,16,3,6,
										 12,17,2,7,
										 11,18,1,8,
										 14,15,4,5,
										 10,19,0,9};//列顺序

uint8_t List_S1[5] = {0,1,0,1,0};
uint8_t List_S2[5] = {0,0,1,1,0};
uint8_t List_S3[5] = {0,0,0,0,1};//0~3
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DMA_Adc_collect(void);
void delay_us(uint32_t us);
void CAN_Send_Data(uint8_t aData[]);
void CAN_Send_Order(uint8_t order);
void VoltageS_2_VoltageD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief CAN过滤器配置函数（对ID进行过滤，只接收0x0000的信息）
  * @retval None
  */
static void CANFilter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
    
    sFilterConfig.FilterBank = 0;                       //CAN过滤器编号，范围0-27
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN过滤器模式，掩码模式或列表模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN过滤器尺度，16位或32位
    sFilterConfig.FilterIdHigh = 0x0000 << 5;			      //32位下，存储要过滤ID的高16位
    sFilterConfig.FilterIdLow = 0x0000;					        //32位下，存储要过滤ID的低16位
    sFilterConfig.FilterMaskIdHigh = 0xFFFF;			      //掩码模式下，存储的是掩码
    sFilterConfig.FilterMaskIdLow = 0xFFFF;
    sFilterConfig.FilterFIFOAssignment = 0;				      //报文通过过滤器的匹配后，存储到哪个FIFO
    sFilterConfig.FilterActivation = ENABLE;    		    //激活过滤器
    sFilterConfig.SlaveStartFilterBank = 0;
    
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);//开启ADC校准
	
	/* 1. CAN Filter Config */
	CANFilter_Config();
    
	/* 2. CAN Start */
	if(HAL_CAN_Start(&hcan) != HAL_OK) 
	{
		Error_Handler();
	}
    
  /* 3. Enable CAN RX Interrupt */
	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) 
	{
		Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Collection_start == 1)//开始数据采集
		{
			DMA_Adc_collect();
			VoltageS_2_VoltageD();
			Collection_start = 0;//采集开始标志置0
			Collection_over = 1;//采集结束标志置1
			SendNum = 0;//数据发送计数位清零
		}
		
		//################从机1取消下方注释，其他从机保持注释#####################
		if(Collection_over == 1)
		{
			CAN_Send_Order(order[0]);//发送采集结束指令
			Collection_over = 0;//采集结束标志置0
		}
				
		if(Send_flag == 1)
		{
			CAN_Send_Data(Voltage_data);//发送采集数据
			Send_flag = 0;//发送标志置0
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief 读取DMA_AD值
  * @retval None
  */
void DMA_Adc_collect(void)
{
	uint8_t r,l,i,n;
	for(l=0; l<15; l++)//列,绿线15
	{
		
		HAL_GPIO_WritePin(GPIOB, A_Pin, Row_A[0]);
		HAL_GPIO_WritePin(GPIOB, B_Pin, Row_B[0]);
		HAL_GPIO_WritePin(GPIOB, C_Pin, Row_C[0]);
		HAL_GPIO_WritePin(GPIOB, D_Pin, Row_D[0]);
		for(r=0; r<5; r++)//行,红线20
		{
			HAL_GPIO_WritePin(GPIOA, S1_Pin, List_S1[0]);
			HAL_GPIO_WritePin(GPIOA, S2_Pin, List_S2[0]);
			HAL_GPIO_WritePin(GPIOA, S3_Pin, List_S3[0]);
			delay_us(50);
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_ConvertedValue, 4);
			delay_us(50);
			for(i=0;i<4;i++)
			{
				n = Lists[r*4+i];//行顺序
				VoltageS[n][l]=(uint16_t)((float)ADC_ConvertedValue[i]/4096*3.300*1000)%10000;
			}
		}
	}
}

/**
  * @brief  数据转换函数
  * @retval None
  */
void VoltageS_2_VoltageD(void)
{
	uint8_t r,l;
	for(l=0; l<15; l++)//列,绿线15
	{
		for(r=0; r<20; r++)//行,红线20
		{
			Voltage_data[l*20+r*2+0] = VoltageS[r][l]>>8;//数据高位
			Voltage_data[l*20+r*2+1] = VoltageS[r][l]&0xff;//数据低位
		}
	}
}

/**
  * @brief 延时函数（单位us）
  * @retval None
  */
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}

/**
  * @brief  CAN 发送数据测试函数
  * @param  aData: 发送的数据
  * @param  len: 数据的长度
  * @retval None
  */
void CAN_Send_Order(uint8_t order)
{
	uint8_t data[1];
	data[0] = order;
		
	uint32_t TxMailbox;
	TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.StdId = 0x0001;       //设置ID号，#########根据从机的序号更改############
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 1;              //设置数据长度

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, data, &TxMailbox) != HAL_OK) 
	{
		Error_Handler();
	}
}

/**
  * @brief  CAN 发送数据测试函数
  * @param  aData: 发送的数据
  * @param  len: 数据的长度
  * @retval None
  */
void CAN_Send_Data(uint8_t aData[])
{
	uint8_t i;
	uint8_t data[8];
	uint32_t TxMailbox;
	
	TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.StdId = 0x0001;       //设置ID号，#########根据从机的序号更改############
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 8;              //设置数据长度

	for(i=0; i<8; i++)//每次发送8个数据
	{
		data[i] = i;//aData[(SendNum-1)*8+i];
	}

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, data, &TxMailbox) != HAL_OK) 
	{
		Error_Handler();
	}
}

/**
  * @brief CAN接收中断处理函数
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t  data[8];

	HAL_StatusTypeDef	status;
	
	status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data);
	if(status == HAL_OK)
	{
		if(RxMessage.StdId == 0x0000)//只接收主机信息
		{
			switch(data[0])
			{
				case 0x09://数据采集指令
					Collection_start = 1;//采集开始标志置0
					break;
				case 0x01://本机地址，#########根据从机的序号更改############
					Send_flag = 1;//发送标志置1
					SendNum++;
					HAL_GPIO_TogglePin(GPIOA,Run_Pin);
					break;
			}
		}
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

