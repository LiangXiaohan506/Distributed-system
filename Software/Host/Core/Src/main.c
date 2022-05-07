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
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w5500.h"
#include "w5500_conf.h"
#include "socket.h"
#include "utility.h"
#include "dhcp.h"
#include "tcp_demo.h"
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

uint8_t order[9] = {0x09,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};//指令集

uint8_t Collection_start = 1;//数据采集开始标志位
uint8_t Collection_over;//数据采集结束标志位
uint8_t order_or_data;//接收数据/指令选择位
uint8_t Slave_x;//从机序号变量
uint8_t Salve_Send_Over;//从机发送结束标志位
uint8_t All_Data_R;//所有数据接收完成标志
uint8_t SlaveX_num;//从机x发送的数据次数

uint8_t Voltage_data[600*8];//发送的阵列传感器数据

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Send(uint8_t orderD);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief CAN过滤器配置函数（掩码全为零，不进行过滤）
  * @retval None
  */
static void CANFilter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
    
    sFilterConfig.FilterBank = 0;                       //CAN过滤器编号，范围0-27
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN过滤器模式，掩码模式或列表模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN过滤器尺度，16位或32位
    sFilterConfig.FilterIdHigh = 0x000 << 5;			      //32位下，存储要过滤ID的高16位
    sFilterConfig.FilterIdLow = 0x0000;					        //32位下，存储要过滤ID的低16位
    sFilterConfig.FilterMaskIdHigh = 0x0000;			      //掩码模式下，存储的是掩码
    sFilterConfig.FilterMaskIdLow = 0x0000;
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
  MX_CAN_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	gpio_for_w5500_config();						/*初始化MCU相关引脚*/
  reset_w5500();                      /* W5500硬件复位 */
	set_w5500_mac();										/*配置MAC地址*/
	set_w5500_ip();											/*配置IP地址*/
	socket_buf_init(txsize, rxsize);		/*初始化8个Socket的发送接收缓存大小*/

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
	
	HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Collection_start == 1)
		{
			CAN_Send(order[0]);//发送采集数据指令
			Collection_start = 0;
		}
		
		if(All_Data_R == 1)
		{
			do_tcp_client(Voltage_data, 600*8);/*TCP_Client 数据回环测试程序*/
			All_Data_R = 0;
			Collection_start = 1;
			HAL_GPIO_TogglePin(GPIOA,TongXin_Pin);
		}

		HAL_GPIO_TogglePin(GPIOA,Run_Pin);
		HAL_Delay(200);
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
}

/* USER CODE BEGIN 4 */
/**
  * @brief CAN 发送数据测试函数
  * @retval None
  */
void CAN_Send(uint8_t orderD)
{
	uint8_t dataO[1];
	dataO[0] = orderD;

	uint32_t TxMailbox;
	TxMessage.IDE = CAN_ID_STD;     //设置ID类型，标准模式
	TxMessage.StdId = 0x0000;        //设置ID号，标识符，可进行过滤
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 1;              //设置数据长度

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, dataO, &TxMailbox) != HAL_OK) 
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
	uint16_t i;
	
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data) == HAL_OK)
	{	
		if(order_or_data == 0)//指令传输
		{
			if(data[0] == 0x09)
			{
					CAN_Send(order[1]);//发送从机1传输指令
					order_or_data = 1;//接收完指令，准备接收数据
			}
		}
		else//数据传输
		{
			switch(RxMessage.StdId)
			{
				case 0x0001://从机1
					Slave_x = 1;
				  SlaveX_num++;  
					for(i=0; i<8; i++)
					{
						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i+1];
					}
					CAN_Send(order[1]);//发送从机2传输指令
					break;
//				case 0x0002://从机2
//					Slave_x = 2;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i+1];
//					}
//					CAN_Send(order[3]);//发送从机3传输指令
//					break;
//				case 0x0003://从机3
//					Slave_x = 3;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[4]);//发送从机4传输指令
//					break;
//				case 0x0004://从机4
//					Slave_x = 4;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[5]);//发送从机5传输指令
//					break;
//				case 0x0005://从机5
//					Slave_x = 5;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[6]);//发送从机6传输指令
//					break;
//				case 0x0006://从机6
//					Slave_x = 6;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[7]);//发送从机7传输指令
//					break;
//				case 0x0007://从机7
//					Slave_x = 7;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[8]);//发送从机8传输指令
//					break;
//				case 0x0008://从机8
//					Slave_x = 3;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i+1];
//					}
//					if(SlaveX_num < 75)
//					{
//						CAN_Send(order[1]);//发送从机1传输指令
//					}else
//					{
//						SlaveX_num=0;
//						All_Data_R = 1;//所有从机数据发送结束
//						order_or_data = 0;//开始接收指令
//					}
//					break;
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

