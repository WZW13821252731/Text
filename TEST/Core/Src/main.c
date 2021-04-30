/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "printfun.h"
#include "delay.h"
#include "sys.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// uint16_t a = 0x12;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define KEY0        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4) 
// uint8_t aTxBuffer[8];      //"*********串口发送数据××××××××××××\r\n";
uint8_t rs485buf[8] ={0X01,0X03,0X00,0X03,0X00,0X01,0X74,0X0A}; 
uint8_t rs485buf3[8] ={0X01,0X03,0X00,0X06,0X00,0X01,0X64,0X0B}; 
uint8_t rs485buf2[8] ={0X01,0X03,0X00,0X00,0X00,0X02,0XC4,0X0B}; 
uint8_t rs1[7];		// 用来接收串口2发送的数据
uint8_t rs2[9];		// 用来接收串口2发送的数据
uint16_t speed1;
uint16_t wind_direction;
uint16_t infrarot;
uint8_t j;
uint8_t data1[10];
uint8_t selet1 = 2;
uint8_t selet2 = 0;
long long temp=0;
long long temp1=0;
//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
uint8_t  TIM5CH1_CAPTURE_STA=0;							//输入捕获状态		    				
uint16_t	TIM5CH1_CAPTURE_VAL;							//输入捕获值(TIM3是16位)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
// int fputc(int ch,FILE *f)
// {
//   //  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
//    HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
//    return ch;
// }

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_Delay(uint32_t Delay){
  delay_ms((uint16_t)Delay);
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
  delay_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2,(uint8_t *)&rs1,7);
  // HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);   //开启TIM3的捕获通道1，并且开启捕获中断
  // HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);   //开启TIM3的捕获通道1，并且开启捕获中断
  __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);   //使能更新中断

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_UART_Receive_IT(&huart2,aRxBuffer2,1);			// Enable the USART2 Interrupt
	// HAL_UART_Transmit(&huart2,aTxBuffer,sizeof(aTxBuffer),100);
  while (1)
  {
    
    switch (selet2)
    {

  case 0:
  
   HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);

    delay_us(10);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
     
    

    if(TIM5CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F; 
			temp*=65536;		 	    	//溢出时间总和
			temp+=TIM5CH1_CAPTURE_VAL;  
      temp= (temp*340); 
      temp = temp /2;   //得到总的高电平时间
      temp = temp /1000000;   //得到总的高电平时间

			printf("距离为:%lld 米\r\n",temp);//打印总的高点平时间
			TIM5CH1_CAPTURE_STA=0;          //开启下一次捕获
		}
    selet2++;
   HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);   //开启TIM3的捕获通道1，并且开启捕获中断


      break;
  case 1:
    
      

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);

    delay_us(10);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
   
    

    if(TIM5CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F; 
			temp*=65536;		 	    	//溢出时间总和
			temp+=TIM5CH1_CAPTURE_VAL;  
      temp= (temp*340); 
      temp = temp /2;   //得到总的高电平时间
      // temp1 = temp1 /1000000;   //得到总的高电平时间

			printf("传感器2距离为:%lld 米\r\n",temp);//打印总的高点平时间
			TIM5CH1_CAPTURE_STA=0;          //开启下一次捕获
		}
    selet2 = 0;
   HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);   //开启TIM3的捕获通道1，并且开启捕获中断

      break;
    
    default:
      break;
    }
    
		
    
    

    switch (selet1)
    {
    case 0/* constant-expression */:
      /* code */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    // uint8_t i;
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)&rs485buf, sizeof(rs485buf), 5000);

     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    HAL_Delay(100);
    println("当前风力等级为：%d级\t\n",data1[4]);
    
    


    // speed =   (int32_t)rs1[4];
    // printf("%02x\t\n",rs1[4]);
    // printf("%02x\t\n",rs1[3]);
    
      // printf("%02x\t\n",data1[0]);
			// printf("%02x\t\n",data1[1]);
			// printf("%02x\t\n",data1[2]);
			// printf("%02x\t\n",data1[3]);
			// printf("%02x\t\n",data1[4]);
			// printf("%02x\t\n",data1[5]);
			// println("%02x\t\n",data1[6]);
      selet1 = 0;
      break;
    case 1/* constant-expression */:
      /* code */
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)&rs485buf2, sizeof(rs485buf2), 5000);

     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    HAL_Delay(100);
    speed1 = data1[3]<<8;
    speed1 += data1[4];
    // speed =   (int32_t)rs1[4];
    println("当前风速为: %d.%d m/s",(speed1/100),(speed1%100));
    wind_direction = (wind_direction|data1[5])<<8;
    wind_direction = (wind_direction|data1[6]);
    // wind_direction = (wind_direction);
    if((0 < wind_direction&& wind_direction < 45 ) || (315 < wind_direction&& wind_direction < 359 ) )
    {
    println("当前风向为北风");

    }else if(45 < wind_direction < 135 )
    {
     println("当前风向为东风");
      

    }else if(135 < wind_direction < 225 )
    {
     println("当前风向为南风");
      

    }
    else if(225 < wind_direction < 315 )
    {
     println("当前风向为西风");
      

    }
    



    // speed =   (int32_t)rs1[4];
    // printf("%02x\t\n",rs1[4]);
    // printf("%02x\t\n",rs1[3]);
    
      // printf("%02x\t\n",data1[0]);
			// printf("%02x\t\n",data1[1]);
			// printf("%02x\t\n",data1[2]);
			// printf("%02x\t\n",data1[3]);
			// printf("%02x\t\n",data1[4]);
			// printf("%02x\t\n",data1[5]);
			// println("%02x\t\n",data1[6]);
    selet1++;
    break;

    case 2 :
    {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    // uint8_t i;
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)&rs485buf3, sizeof(rs485buf3), 5000);

     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    HAL_Delay(100);
  

    infrarot = (infrarot|data1[3])<<8;
    infrarot = (infrarot|data1[4]);

   println("当前隧道外亮度为: %d Lux",infrarot);
   infrarot = 0;
 
  //  println("当前隧道外亮度为: %d.%d Lux",infrarot,(infrarot%10));
  


    }
      selet1  = 0;
      break;
    
    default:
      break;
    }
    
      
    


    // HAL_Delay(10);
    // HAL_UART_Transmit(&huart2, (uint8_t*)&rs485buf2, sizeof(rs485buf), 5000);


    
    // for(i =0;i<7;i++)
    // {
    //   printf("0x%02x\t",rs1[i]);
    // }
    // printf("\r\n");
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 4800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  
  switch (selet1)
  {
  case  0/* constant-expression */:
    /* code */
    for(j = 0;j<7;j++)
  {
    data1[j]=rs1[j];
  }
    HAL_UART_Receive_IT(&huart2,(uint8_t *)&rs2,9);

    break;
  case  1/* constant-expression */:
   for(j = 0;j<9;j++)
  {
    data1[j]=rs2[j];
  }
    HAL_UART_Receive_IT(&huart2,(uint8_t *)&rs1,7);
    /* code */
    break;
  case 2/* constant-expression */:
    for(j = 0;j<7;j++)
  {
    data1[j]=rs1[j];
  }
    HAL_UART_Receive_IT(&huart2,(uint8_t *)&rs1,7);
    break;

  default:
    break;
  }
  

  // HAL_UART_Transmit(&huart1,rs1,7,1000);
  //   //HAL_UART_Transmit(&huart1, (uint8_t*)&aRxBuffer2, sizeof(aRxBuffer2), 1000);
	// 	while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发送结束

}
//定时器更新中断（计数溢出）中断处理回调函数， 该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//更新中断（溢出）发生时执行
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)				//还未成功捕获
	{
		if(TIM5CH1_CAPTURE_STA&0X40)				//已经捕获到高电平了
		{
			if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)	//高电平太长了
			{
				TIM5CH1_CAPTURE_STA|=0X80;			//标记成功捕获了一次
				TIM5CH1_CAPTURE_VAL=0XFFFF;
			}else TIM5CH1_CAPTURE_STA++;
		}	 
	}	
  
	
}

//定时器输入捕获中断处理回调函数，该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)				//还未成功捕获
	{
		if(TIM5CH1_CAPTURE_STA&0X40)				//捕获到一个下降沿 		
		{	  			
			TIM5CH1_CAPTURE_STA|=0X80;				//标记成功捕获到一次高电平脉宽
            TIM5CH1_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);//获取当前的捕获值.
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
            TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//配置TIM3通道1上升沿捕获
      

		}else  										//还未开始,第一次捕获上升沿
		{
			TIM5CH1_CAPTURE_STA=0;					//清空
			TIM5CH1_CAPTURE_VAL=0;
			TIM5CH1_CAPTURE_STA|=0X40;				//标记捕获到了上升沿
			__HAL_TIM_DISABLE(&htim3);      	//关闭定时器3
			__HAL_TIM_SET_COUNTER(&htim3,0);
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//定时器3通道1设置为下降沿捕获
			__HAL_TIM_ENABLE(&htim3);		//使能定时器3
      

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
