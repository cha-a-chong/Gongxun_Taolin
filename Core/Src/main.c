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
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Arm/Arm.h"
#include "Run/Run.h"
#include "Emm_V5/Emm_V5.h"
#include "Action/Action.h"
#include "Screen/Screen.h"
#include "Joy_Interface/Joy_Interface.h"
#include "Location/Location.h"
#include "stdio.h"
#include "string.h"
#include "SCSLib/SCServo.h"
#include "SCSLib/SCS_Uart.h"
#include "HWT101CT/HWT101CT.h"

#include "Stability/Stability.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RunSpeed 100
#define RunAcc 120
#define yanshi 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*  ---- BUG Log Start ----
 BUG:
 * A.在抓取物料阶段，如果没有TX2物料坐标，则PID会有问题
 * 	 解决方案：使用标志位，如果没有TX2坐标返回标志，则不进入PID调整阶段，�?�是进入等待
 * 	 目前状�?�：有点小Bug,上面的情况只是会偶尔出现
 * B.屏幕上没有显示TX2回传物料坐标点，不利于调�?
 * 	 解决进度: 已解�?
 * C.屏幕显示物料颜色
 * 	 解决方案：是修改串口屏工程，显示�?整块矩形色块
 * 	 解决进度：可以显示当前识别物料及色环的颜�?
 * D.物料旋转过程中，没有判定物料是否会停下，而是在检测到后直接进行抓�?
 * 	 解决方案：修�?32端代码，取十次数据差值，在差值趋�?1%精度时视为稳�?
 * 	 解决进度：写了，好像没有�?么用，进�?1%
 * E.在TX2识别过程中不会主动滤色环或�?�物料，而是�?测到�?么就发�?�什么，�?要�?�过32端主动过滤，防止影响控制
 * 	 解决进度：新建文件夹
 FIXME:
 修改了爪子支撑结构中的飞特舵机零位，后面与飞特舵机有关的点位都需要调�?
 解决进度：调整到了第�?次抓�?
 * ---- Bug Log End ------
 */

//	串口接受数据变量
uint8_t data;
uint8_t Screen_data;
uint8_t Joy_data = 0;
uint8_t HwtData;
// TX2 接收数据变量
uint8_t aRxBuffer = 0;
char RxBuffer[RXBUFFERSIZE] =
{ 0 };
uint8_t Uart10_Rx_Cnt = 0;
float x = .0;
float y = .0;
char Point_Flag = 0;
int QR_data[6] =
{ 0, 0, 0, 0, 0, 0 };
// PID设定目标�?
float x_goal, y_goal, a_goal;
// TODO：需要根据调试过程修改目标�??
float tx_target = 339;
float ty_target = 227;
uint16_t flag = 0;
uint16_t TX_flag = 0;
int colour;
int wuliao_falg;

int QR_Flag = 0;

// 正式比赛前车身姿态调整标志位
extern uint8_t Ready_Flag;
// 抓取过程标志�?
char Match_Flag = 0;
// 抓取计算稳定性标志位�?0为识别有故障,1为正常计�?
char Check_flag = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
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

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	//  USART10 用于Action串口转发,暂时使用无线串口将数据转发到主机
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_FDCAN1_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_UART9_Init();
	MX_USART6_UART_Init();
	MX_USART10_UART_Init();
	MX_TIM2_Init();
	MX_USART3_UART_Init();
	MX_UART7_Init();
	MX_UART8_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_USART2_UART_Init();
	MX_TIM12_Init();
	/* USER CODE BEGIN 2 */

//	清除定时器初始化过程中的更新中断标志�?
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
//	使能定时器中�?
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim3);
//	使能串口接收中断
	HAL_UART_Receive_IT(&huart4, &Screen_data, 1);
//	HWT101预留中断接口
	HAL_UART_Receive_IT(&huart2, &HwtData, 1);
//	TX2通信接收中断
	HAL_UART_Receive_IT(&huart10, (uint8_t*) &aRxBuffer, 1);
	HAL_UART_Receive_IT(&huart9, &data, 1);
//	HAL_UART_Receive_IT(&huart10, &Joy_data, 1);

	PID_Init();
//  延时2s用于等待步进电机上电
//	HAL_Delay(2000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//	while(1)
		//	{
		//		if(Joy_Flag == 1)
		//			break;
		//		else if (System_Flag == 1)
		//		{
		////			HAL_TIM_Base_Start_IT(&htim3);
		////		    初始化PID参数
		//			PID_Init();
		//			break;
		//		}
		//	}
//		while (1) {
//			if (Joy_Flag == 1) {
//				Check_Joydata();
//				HAL_Delay(100);
//			}
//		}
		if (Ready_Flag == 1)
		{
			Ready_Flag = 0;
//			整定车身
			Check_Status();
		}
		if (System_Flag == 1)
		{
			System_Flag = 0;
//			向TX2发�?�字符串"e1f"
			HAL_UART_Transmit(&huart10, (uint8_t*) "e1f", sizeof("elf") - 1,
					0x1000);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//			Start();  //步进电机在最低处
//			catch_Frist(860, 2375);  //上下同等
//			catch_Frist(1910, 2370);  //上下同等
//			catch_Frist(2960, 2370);  //上下同等
//			Drop_Location_jiang(50, 50, 11000);
//			put(2150, 2680);
//			HAL_Delay(2000);
//			put(3205, 2680);
//			HAL_Delay(2000);
//
//			put(2960, 2370);
//			HAL_Delay(2000);
//			catch_Second(860, 2375);
//			catch_Second(1910, 2370);  //上下同等
//			catch_Second(2960, 2370);  //上下同等
//			while(1);
//			//
//			while(1);

//			x_goal = 100;
//
//			y_goal = 50;
//
//			while (1) {               //ACTION调整
//				if ((X_NOW - x_goal) < 0) {
//						Move_Left(100, 100, ABS(X_NOW - x_goal)*13.8);
//						HAL_Delay(yanshi);
//					} else {
//						Move_Right(100, 100, ABS(X_NOW - x_goal)*13.8);
//						HAL_Delay(yanshi);
//					}
//					if (Y_NOW - y_goal < 0) {
//						Move_Line(100, 100, ABS(Y_NOW - y_goal)*13.8);
//						HAL_Delay(yanshi);
//					} else {
//						Move_Back(100, 100, ABS(Y_NOW - y_goal)*13.8);
//						HAL_Delay(yanshi);
//					}
//
//				if ( ABS(X_NOW - x_goal) < 2 && ABS(Y_NOW - y_goal) < 2) {
//					break; /* 到达目标 */
//				}
//			}
//			Move_Xiajiang(100,100,14000);
//			HAL_Delay(2000);

//			Move_Left(50,50,8000);
///////////////////////////////////////////////////////////////////////////////////////////////////////////
			/***********************运行逻辑-状�?�机*************************/
			while (1)
			{
				switch (flag)
				{
				case 0:  //发车，进入扫码区
//					X轴建议移动到150左右
					// Move_TO_Saomaqu(2400, 8250);
					bool temp = Move_Left(RunSpeed, RunAcc, 2400);
					while(temp != true)
					{
						temp = Move_Left(RunSpeed, RunAcc, 2400);
					}    
					Start();    //5400�?
					flag = 1;
					temp=false;
					break;
//					Drop_Location_jiang(200, 120, 4000);
				case 1:  //离开扫码区，进入原料�?
//					物料理想点位 375
					temp = Move_Line(RunSpeed, RunAcc, 10500);
					while(temp != true)
					{
						temp = Move_Line(RunSpeed, RunAcc, 10500);
					}
//					Move_TO_yuanliaoqu(10500);
//					Choke_Flag = false确定Move_Line已经执行完毕，可以开始进入PID整定阶段
					while(Choke_Flag == true)
					{
						;
					}
//					X参�??150 Y参�??1430
					Move_Action_Nopid_Left_Ctrl(150, 1455);  //ACTION调整
//					�?查TX2是否传�?�物料�??
					while (Point_Flag != 1)
					{
						;  //如果Point_Flag值不�?1,则在此处等待
					}
//					TODO:应该在物料抓取函数中实现基于TX2的物料点位闭�?
					Frist_Grab_Wuliao();
					flag = 2;
					break;

				case 2:  //离开原料区，进入加工�?

					Move_TO_jianzhi1(4500, 4335);
					while (1)
					{
						;
					}
//					Drop_Location_jiang(50, 50, 11000);
					HAL_Delay(50);
					Move_Action_Nopid_Forward_Ctrl(160, 1070);
					flag = 3;
					break;
				case 3:    //暂存�??

					Move_TO_zancunqu(22000, 4335);
					put_wuliao_to_circular_frist();
					put_wuliao_to_Car_frist();
					HAL_Delay(yanshi);
					flag = 4;
					break;
				case 4:
					Move_TO_jianzhi2(9000, 4335);     //ACTION调整
					Move_Action_Nopid_Forward_Ctrl(1870, 1860);
					flag = 5;
					break;
				case 5:       //粗加工区
					Move_TO_cujiagongqu(10000);
					put_wuliao_to_circular_second();
					flag = 6;
					break;
				case 6:
					Move_TO_jianzhi3(9000, 4335);
					Move_Action_Nopid_Left_Ctrl(170, 1860);
					Drop_Location_jiang(50, 50, 11000);
					flag = 7;
					break;
				case 7:      //原料�??
					Move_TO_fanyuanliaoqu(4000);
					Move_Action_Nopid_Left_Ctrl(160, 1400);      //ACTION调整
					Second_Run_Frist_Grab_Wuliao();
					flag = 8;
					break;
				case 8:
					Move_TO_jianzhi1(4500, 4335);
					Drop_Location_jiang(50, 50, 11000);
					Move_Action_Nopid_Forward_Ctrl(160, 1070);
					flag = 9;
					break;
				case 9:    //暂存�??
					Move_TO_zancunqu(22000, 4335);
					Second_Run_put_wuliao_to_circular_frist();

					Second_Run_put_wuliao_to_Car_frist();
					HAL_Delay(yanshi);
					flag = 10;
					break;
				case 10:
					Move_TO_jianzhi2(9000, 4335);     //ACTION调整
					Move_Action_Nopid_Forward_Ctrl(1870, 1860);
					flag = 11;
					break;
				case 11:       //粗加工区
					Move_TO_cujiagongqu(10000);

					flag = 12;
					break;
				case 12:
					Move_TO_jianzhi3(9000, 4335);
					Move_Action_Nopid_Left_Ctrl(170, 1860);
					flag = 13;
					break;
				case 13:      //原料�??
					Move_TO_fanyuanliaoqu(22000);
					flag = 14;
					break;
				default:
					Move_Stop();
					break;
				}
			}
			if (Ready_Flag == 1)
			{
				Ready_Flag = 0;
				//			整定车身
				Check_Status();
			}
			/*********************************纯跑�??*******************************************/
//
//			while (1) {
//
//				switch (flag) {
//				case 0:  //扫码�??
//					Move_TO_Saomaqu(2100, 7500);
//					flag = 1;
//					break;
//				case 1:  //原料�??
//					Move_TO_yuanliaoqu(10500);
//					Move_Action_Nopid_Left_Ctrl(160, 1400);  //ACTION调整
//					flag = 2;
//					break;
//
//				case 2:
//					Move_TO_jianzhi1(4500, 4335);
//					Move_Action_Nopid_Forward_Ctrl(160, 1070);
//					flag = 3;
//					break;
//				case 3:
//					Move_TO_zancunqu(22000, 4335);
//
//					Move_Action_Nopid_Right_Ctrl(1900, 1050);
//					HAL_Delay(2000);
//					Move_Action_Nopid_Right_Ctrl(1900, 885 );
//					HAL_Delay(2000);
//					Move_Action_Nopid_Right_Ctrl(1900, 1185);
//					HAL_Delay(2000);
//					flag = 4;
//					break;
//				case 4:
//					Move_TO_jianzhi2(9000, 4335);     //ACTION调整
//					Move_Action_Nopid_Forward_Ctrl(1870, 1860);
//					flag = 5;
//					break;
//				case 5:
//					Move_TO_cujiagongqu(10000);
//
//					Move_Action_Nopid_Forward_Ctrl(1065, 1910);
//					HAL_Delay(2000);
//					Move_Action_Nopid_Forward_Ctrl(1185, 1910);
//					HAL_Delay(2000);
//					Move_Action_Nopid_Forward_Ctrl(910, 1910);
//					HAL_Delay(2000);
//					flag = 6;
//					break;
//				case 6:
//					Move_TO_jianzhi3(9000, 4335);
//					Move_Action_Nopid_Left_Ctrl(170, 1860);
//					flag = 7;
//					break;
//				case 7:
//					//原料�??
//					Move_TO_fanyuanliaoqu(5000);
//					Move_Action_Nopid_Left_Ctrl(160, 1400);      //ACTION调整
//					flag = 8;
//					break;
//				default:
//					flag = 0;
//					break;
//				}
//			}
/////////////////////////////////////////////////////////////////////////////////
			/**************************抓取参数*********************************************/
//					catch(860, 2375);
//					catch(1910, 2370);
//					catch(2960, 2370);
//					put(2150, 2680);
//					HAL_Delay(2000);
//					put(3205, 2680);
//					HAL_Delay(2000);
//					put(2960, 2370);
//					HAL_Delay(2000);
//  			    put(1910,2370);
//					HAL_Delay(2000);
//				    put(860,2370);
//					HAL_Delay(2000);
//					put(2960, 2370);
//					HAL_Delay(2000);
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
	{
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 44;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart4, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART9)                          // 判断中断�?
	{
		Data_Analyse(data);                                // 处理Action数据
		HAL_UART_Receive_IT(&huart9, &data, 1);            // 继续中断回调

	}
	else if (huart->Instance == UART4)                   // 判断中断�?
	{
		Check_Flag(Screen_data);                       	   // 处理串口屏数�?
		HAL_UART_Receive_IT(&huart4, &Screen_data, 1);     // 继续中断回调
	}

	else if (huart->Instance == USART10)
	{
		//static uint8_t RxState = 0;

		RxBuffer[Uart10_Rx_Cnt++] = aRxBuffer;

		if (Uart10_Rx_Cnt >= sizeof(RxBuffer))
		{
			memset(RxBuffer, 0, sizeof(RxBuffer));
			Uart10_Rx_Cnt = 0;
			return;
		}

		if (aRxBuffer == 'd')
		{ // 如果包尾�?'d'，说明是物料和圆环点位数�?
			char *start = strchr(RxBuffer, 'a');
			char *end = strchr(RxBuffer, 'd');

			if (start != NULL && end != NULL && end > start)
			{
				*end = '\0';
				if (sscanf(start + 1, "%f,%f,%d", &x, &y, &colour) == 3)
				{
					// 数据解析成功
					Point_Flag = 1;
				}

				// 清空缓冲�?
				memset(RxBuffer, 0, sizeof(RxBuffer));
				Uart10_Rx_Cnt = 0;
			}
		}

		if (aRxBuffer == 'b')
		{ // 如果包尾�?'b'，说明是二维码数�?
			char *start = strchr(RxBuffer, 'c');
			char *end = strchr(RxBuffer, 'b');

			if (start != NULL && end != NULL && end > start)
			{
				*end = '\0';
//				for (int i = 0; i < 6; i++) {
//					if (sscanf(start +1+ i, "%d", &QR_data[i])) {
				if (sscanf(start + 1, "%d,%d,%d,%d,%d,%d", &QR_data[0],
						&QR_data[1], &QR_data[2], &QR_data[3], &QR_data[4],
						&QR_data[5]))
				{
					// 数据解析成功
//						if(i==5) QR_Flag =  1;
					QR_Flag = 1;
				}
//				}
				// 清空缓冲�?
				memset(RxBuffer, 0, sizeof(RxBuffer));
				Uart10_Rx_Cnt = 0;
			}
		}
		// 重新启动接收
		HAL_UART_Receive_IT(&huart10, (uint8_t*) &aRxBuffer, 1);
	}
//	else if (huart->Instance == USART2)                            // 判断中断
//	{
//
//		static uint8_t rx_buffer[11];
//		static uint8_t rx_index = 0;
//		static uint8_t state = 0; // 0: 寻找包头�??1: 接收数据
//
//		if (state == 0) { // 寻找包头
//
//			if (HwtData == 0x55) {
//				rx_buffer[0] = HwtData;
//				rx_index = 1;
//				state = 1;
//			}
//		} else if (state == 1) { // 接收数据
//			rx_buffer[rx_index++] = HwtData;
//			if (rx_index == 2 && rx_buffer[1] != 0x53 && rx_buffer[1] != 0x52) { // 如果第二字节不是0x53�??0x52，则继续寻找下一�??0x55
//				if (rx_buffer[1] == 0x55) {
//					rx_index = 1;
//				} else {
//					state = 0;
//					rx_index = 0;
//				}
//			} else if (rx_index == 11) { // 接收到了完整的数据包
//				ParseAndPrintData(rx_buffer, 11);
//				rx_index = 0;
//				state = 0; // 重新寻找包头
//			}
//		}
//		HAL_UART_Receive_IT(&huart2, &HwtData, 1);
//
//	}
//
//	else if (huart->Instance == USART10)                            // 判断中断
//	{
//		Recive_Joydata(Joy_data);                             	// 处理Joy_data数据
//		HAL_UART_Receive_IT(&huart10, &Joy_data, 1);       // 继续中断回调
//	}
}
extern uint16_t time_tx;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{

//		X_out = X_Ctrl(&XPID, x_goal);
//		Y_out = Y_Ctrl(&YPID, y_goal);
//		A_out = A_Ctrl(&APID, a_goal);
		Action_Show();
		QR_Show();
		Point_Show();

	}
	else if (htim == &htim3)
	{
		TX_X_out = Tx_X_Ctrl(&TXPID, tx_target);
		TX_Y_out = Tx_Y_Ctrl(&TYPID, ty_target);
		time_tx++;
	}
	else if (htim == &htim5)
	{

		if (time5_jiancha != 0)
			time5_jiancha--;
	}
//	步进电机阻塞操作回调
	else if (htim == &htim12)
	{
		if(Apply_Chock == true)
		{
			if(Base_Data == true)
			{
				Choke_Flag = false;
				Apply_Chock = false;
				HAL_TIM_Base_Stop_IT(&htim12);
				Base_Data = false;
			}
			else
			{
				return ;
			}
		}
//		根据查询情况来确定是否要继续阻塞 Choke_Flag
	}
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct =
	{ 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
