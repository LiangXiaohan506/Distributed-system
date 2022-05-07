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
static CAN_TxHeaderTypeDef        TxMessage;    //CAN���͵���Ϣ����Ϣͷ
static CAN_RxHeaderTypeDef        RxMessage;    //CAN���յ���Ϣ����Ϣͷ

uint8_t order[9] = {0x09,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};//ָ�

uint8_t Collection_start = 1;//���ݲɼ���ʼ��־λ
uint8_t Collection_over;//���ݲɼ�������־λ
uint8_t order_or_data;//��������/ָ��ѡ��λ
uint8_t Slave_x;//�ӻ���ű���
uint8_t Salve_Send_Over;//�ӻ����ͽ�����־λ
uint8_t All_Data_R;//�������ݽ�����ɱ�־
uint8_t SlaveX_num;//�ӻ�x���͵����ݴ���

uint8_t Voltage_data[600*8];//���͵����д���������

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Send(uint8_t orderD);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief CAN���������ú���������ȫΪ�㣬�����й��ˣ�
  * @retval None
  */
static void CANFilter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
    
    sFilterConfig.FilterBank = 0;                       //CAN��������ţ���Χ0-27
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN������ģʽ������ģʽ���б�ģʽ
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN�������߶ȣ�16λ��32λ
    sFilterConfig.FilterIdHigh = 0x000 << 5;			      //32λ�£��洢Ҫ����ID�ĸ�16λ
    sFilterConfig.FilterIdLow = 0x0000;					        //32λ�£��洢Ҫ����ID�ĵ�16λ
    sFilterConfig.FilterMaskIdHigh = 0x0000;			      //����ģʽ�£��洢��������
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = 0;				      //����ͨ����������ƥ��󣬴洢���ĸ�FIFO
    sFilterConfig.FilterActivation = ENABLE;    		    //���������
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
	gpio_for_w5500_config();						/*��ʼ��MCU�������*/
  reset_w5500();                      /* W5500Ӳ����λ */
	set_w5500_mac();										/*����MAC��ַ*/
	set_w5500_ip();											/*����IP��ַ*/
	socket_buf_init(txsize, rxsize);		/*��ʼ��8��Socket�ķ��ͽ��ջ����С*/

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
			CAN_Send(order[0]);//���Ͳɼ�����ָ��
			Collection_start = 0;
		}
		
		if(All_Data_R == 1)
		{
			do_tcp_client(Voltage_data, 600*8);/*TCP_Client ���ݻػ����Գ���*/
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
  * @brief CAN �������ݲ��Ժ���
  * @retval None
  */
void CAN_Send(uint8_t orderD)
{
	uint8_t dataO[1];
	dataO[0] = orderD;

	uint32_t TxMailbox;
	TxMessage.IDE = CAN_ID_STD;     //����ID���ͣ���׼ģʽ
	TxMessage.StdId = 0x0000;        //����ID�ţ���ʶ�����ɽ��й���
	TxMessage.RTR = CAN_RTR_DATA;   //���ô�������֡
	TxMessage.DLC = 1;              //�������ݳ���

	if (HAL_CAN_AddTxMessage(&hcan, &TxMessage, dataO, &TxMailbox) != HAL_OK) 
	{
		Error_Handler();
  }
}

/**
  * @brief CAN�����жϴ�����
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t  data[8];
	uint16_t i;
	
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data) == HAL_OK)
	{	
		if(order_or_data == 0)//ָ���
		{
			if(data[0] == 0x09)
			{
					CAN_Send(order[1]);//���ʹӻ�1����ָ��
					order_or_data = 1;//������ָ�׼����������
			}
		}
		else//���ݴ���
		{
			switch(RxMessage.StdId)
			{
				case 0x0001://�ӻ�1
					Slave_x = 1;
				  SlaveX_num++;  
					for(i=0; i<8; i++)
					{
						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i+1];
					}
					CAN_Send(order[1]);//���ʹӻ�2����ָ��
					break;
//				case 0x0002://�ӻ�2
//					Slave_x = 2;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i+1];
//					}
//					CAN_Send(order[3]);//���ʹӻ�3����ָ��
//					break;
//				case 0x0003://�ӻ�3
//					Slave_x = 3;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[4]);//���ʹӻ�4����ָ��
//					break;
//				case 0x0004://�ӻ�4
//					Slave_x = 4;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[5]);//���ʹӻ�5����ָ��
//					break;
//				case 0x0005://�ӻ�5
//					Slave_x = 5;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[6]);//���ʹӻ�6����ָ��
//					break;
//				case 0x0006://�ӻ�6
//					Slave_x = 6;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[7]);//���ʹӻ�7����ָ��
//					break;
//				case 0x0007://�ӻ�7
//					Slave_x = 7;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i];
//					}
//					CAN_Send(order[8]);//���ʹӻ�8����ָ��
//					break;
//				case 0x0008://�ӻ�8
//					Slave_x = 3;
//					for(i=0; i<8; i++)
//					{
//						Voltage_data[(Slave_x-1)*600+(SlaveX_num-1)*8+i] = data[i+1];
//					}
//					if(SlaveX_num < 75)
//					{
//						CAN_Send(order[1]);//���ʹӻ�1����ָ��
//					}else
//					{
//						SlaveX_num=0;
//						All_Data_R = 1;//���дӻ����ݷ��ͽ���
//						order_or_data = 0;//��ʼ����ָ��
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

