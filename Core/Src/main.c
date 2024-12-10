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

/*  ---- BUG Log ----
    ----  标记符号规范
// TODO: 用来标记待办的地方。常常在有些地方，我们的功能并没有实现，使用ToDo标记我们可以快速定位需要实现的部分。
// HACK: 用来标记可能需要更改的地方。在写代码的时候，有的地方我们并不确定他是正确的，可能未来有所更改，这时候可以使用HACK标记。
// NOTE: 添加一些说明文字。
// INFO: 用来表达一些信息。
// TAG: 用来创建一些标记。
// XXX: 用来标记一些草率实现的地方。在写代码的时候，有些地方需要频繁修改，这时候使用XXX标记。
// BUG: 用来标记BUG~
// FIXME: 用来标记一些需要修复的位置，可以快速定位。

 ------- History -------

- 2024/12/11 : 修改main.c中乱乱码的注释字符串



 BUG:
 * A. 在抓取物料阶段，如果没有TX2物料坐标，则PID会有问题
 * 	  解决方案：使用标志位，如果没有TX2坐标返回标志，则不进入PID调整阶段，应当进入等待
 * 	  目前状况：有点小Bug,上面的情况只是会偶尔出现
 *
 * B. 屏幕上没有显示TX2回传物料坐标点，不利于调试
 * 	  解决进度: 已解决
 *
 * C. 屏幕显示物料颜色
 * 	  解决方案：修改串口屏工程，
 * 	  解决进度：可以显示当前识别物料及色环的颜色代码(1:red 2:green 3:blue 5:?)
 *
 * D. 物料旋转过程中，没有判定物料是否会停下，而是在检测到后直接进行抓取
 * 	  解决方案：修改32端代码，取十次数据差值，在差值趋于1%精度时视为进入稳态
 * 	  解决进度：写了，好像没有什么用，进度1%
 *
 * E. 在TX2识别过程中不会主动滤色环或�?�物料，而是�?测到�?么就发�?�什么，�?要�?�过32端主动过滤，防止影响控制
 * 	  解决进度：新建文件夹
 *
 * F. 步进电机失步，未走完则到位返回？应该是执行到了下一步 --桃林饮酒
 * 	  目前状况: 根据debug过程中的调试,在第一次向左移动后,出现步进电机卡住，但是步进电机阻塞操作标志位为true,也就是说没有返回到位标志位。
 * 	  解决方案: Emm_V5中增加查询电机状态的函数()
 * G. 第一步向左走然后停止的频率较高，目前猜测是未收到到位返回命令
 *
 * H. 原料区识别抓取存在问题
 *
 * I. 刚开启启动调整车身  舵机不动
 FIXME:
 修改了爪子支撑结构中的飞特舵机零位，后面与飞特舵机有关的点位都需要调�?
 解决进度：调整到了第一次抓取过程
 Hello World  泥嚎
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
// PID�趨Ŀ��??
float x_goal, y_goal, a_goal;
// TODO����Ҫ���ݵ��Թ����޸�Ŀ��???
float tx_target = 339;
float ty_target = 227;
uint16_t flag = 0;
uint16_t TX_flag = 0;
int colour;
int wuliao_falg;

int QR_Flag = 0;

// ��ʽ����ǰ������̬������־λ
extern uint8_t Ready_Flag;
// ץȡ���̱�־??
char Match_Flag = 0;
// ץȡ�����ȶ��Ա�־λ??0Ϊʶ���й���,1Ϊ������??
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
	//  USART10 ����Action����ת��,��ʱʹ�����ߴ��ڽ�����ת��������
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

//	�����ʱ����ʼ�������еĸ����жϱ�־??
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
//	ʹ�ܶ�ʱ����??
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim3);
//	ʹ�ܴ��ڽ����ж�
	HAL_UART_Receive_IT(&huart4, &Screen_data, 1);
//	HWT101Ԥ���жϽӿ�
	HAL_UART_Receive_IT(&huart2, &HwtData, 1);
//	TX2ͨ�Ž����ж�
	HAL_UART_Receive_IT(&huart10, (uint8_t*) &aRxBuffer, 1);
	HAL_UART_Receive_IT(&huart9, &data, 1);
//	HAL_UART_Receive_IT(&huart10, &Joy_data, 1);

	PID_Init();
//  ��ʱ2s���ڵȴ���������ϵ�
//	HAL_Delay(2000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
// 		����׼�����������öκ���
		if (Ready_Flag == 1)
		{
			Ready_Flag = 0;
//			��������
			Check_Status();
		}
// 		���·��������öκ���
		if (System_Flag == 1)
		{
			System_Flag = 0;
//			��TX2�����ַ���"e1f"
			HAL_UART_Transmit(&huart10, (uint8_t*) "e1f", sizeof("elf") - 1,
					0x1000);
			/***********************���������߼�״̬��*************************/
			while (1)
			{
				switch (flag)
				{
				case 0:  //����������ɨ����
//					X�Ὠ���ƶ���150����
				// Move_TO_Saomaqu(2400, 8250);
					bool temp = Move_Left(RunSpeed, RunAcc, 2400);
					while (temp != true)
					{
						temp = Move_Left(RunSpeed, RunAcc, 2400);
					}
					Start();
					flag = 1;
					break;
//					Drop_Location_jiang(200, 120, 4000);
				case 1:  //�뿪ɨ����������ԭ��??
//					���������λ 375
					temp = Move_Line(RunSpeed, RunAcc, 10500);
					while (temp != true)
					{
// 						����öα���������һֱ�ط�
						temp = Move_Line(RunSpeed, RunAcc, 10500);
					}
//					Choke_Flag = falseȷ��Move_Line�Ѿ�ִ����ϣ����Կ�ʼ����PID�����׶�
					while (Choke_Flag == true)
					{
						;
					}
//					Action��λ���� X����150 Y����1455
//					Move_Action_Nopid_Left_Ctrl(150, 1455);
//					���TX2�Ƿ񴫵����ϵ�λ��Ϣ
					while (Point_Flag != 1)
					{
						;  //���Point_Flagֵ��Ϊ1,��û��ʶ�����ϣ������ȴ�״̬
					}
//					TODO:Ӧ��������ץȡ������ʵ�ֻ���TX2�����ϵ�λ�ջ�
//					Frist_Grab_Wuliao();
					flag = 2;
					break;

				case 2:  //�뿪ԭ����������ʮ��ǰ����������̬У׼
					Move_TO_jianzhi1(4500, 4335);
					Roll_Status();
					HAL_Delay(50);
//					Move_Action_Nopid_Forward_Ctrl(160, 1070);
					flag = 3;
					break;
				case 3:    //�ݴ�???
					Move_TO_zancunqu(22000, 4335);
					while (1)
					{
						;
					}
					put_wuliao_to_circular_frist();
					put_wuliao_to_Car_frist();
					HAL_Delay(yanshi);
					flag = 4;
					break;
				case 4:
					Move_TO_jianzhi2(9000, 4335);     //ACTION����
					Move_Action_Nopid_Forward_Ctrl(1870, 1860);
					flag = 5;
					break;
				case 5:       //�ּӹ���
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
				case 7:      //ԭ��???
					Move_TO_fanyuanliaoqu(4000);
					Move_Action_Nopid_Left_Ctrl(160, 1400);      //ACTION����
					Second_Run_Frist_Grab_Wuliao();
					flag = 8;
					break;
				case 8:
					Move_TO_jianzhi1(4500, 4335);
					Drop_Location_jiang(50, 50, 11000);
					Move_Action_Nopid_Forward_Ctrl(160, 1070);
					flag = 9;
					break;
				case 9:    //�ݴ�???
					Move_TO_zancunqu(22000, 4335);
					Second_Run_put_wuliao_to_circular_frist();

					Second_Run_put_wuliao_to_Car_frist();
					HAL_Delay(yanshi);
					flag = 10;
					break;
				case 10:
					Move_TO_jianzhi2(9000, 4335);     //ACTION����
					Move_Action_Nopid_Forward_Ctrl(1870, 1860);
					flag = 11;
					break;
				case 11:       //�ּӹ���
					Move_TO_cujiagongqu(10000);

					flag = 12;
					break;
				case 12:
					Move_TO_jianzhi3(9000, 4335);
					Move_Action_Nopid_Left_Ctrl(170, 1860);
					flag = 13;
					break;
				case 13:      //ԭ��???
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
				//			��������
				Check_Status();
			}
			/*********************************����???*******************************************/
//
//			while (1) {
//
//				switch (flag) {
//				case 0:  //ɨ��???
//					Move_TO_Saomaqu(2100, 7500);
//					flag = 1;
//					break;
//				case 1:  //ԭ��???
//					Move_TO_yuanliaoqu(10500);
//					Move_Action_Nopid_Left_Ctrl(160, 1400);  //ACTION����
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
//					Move_TO_jianzhi2(9000, 4335);     //ACTION����
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
//					//ԭ��???
//					Move_TO_fanyuanliaoqu(5000);
//					Move_Action_Nopid_Left_Ctrl(160, 1400);      //ACTION����
//					flag = 8;
//					break;
//				default:
//					flag = 0;
//					break;
//				}
//			}
/////////////////////////////////////////////////////////////////////////////////
			/**************************ץȡ����*********************************************/
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
	if (huart->Instance == UART9)                          // �ж��ж�??
	{
		Data_Analyse(data);                                // ����Action����
		HAL_UART_Receive_IT(&huart9, &data, 1);            // �����жϻص�

	}
	else if (huart->Instance == UART4)                   // �ж��ж�??
	{
		Check_Flag(Screen_data);                       	   // ����������??
		HAL_UART_Receive_IT(&huart4, &Screen_data, 1);     // �����жϻص�
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
		{ // �����β??'d'��˵�������Ϻ�Բ����λ��??
			char *start = strchr(RxBuffer, 'a');
			char *end = strchr(RxBuffer, 'd');

			if (start != NULL && end != NULL && end > start)
			{
				*end = '\0';
				if (sscanf(start + 1, "%f,%f,%d", &x, &y, &colour) == 3)
				{
					// ���ݽ����ɹ�
					Point_Flag = 1;
				}

				// ��ջ���??
				memset(RxBuffer, 0, sizeof(RxBuffer));
				Uart10_Rx_Cnt = 0;
			}
		}

		if (aRxBuffer == 'b')
		{ // �����β??'b'��˵���Ƕ�ά����??
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
					// ���ݽ����ɹ�
//						if(i==5) QR_Flag =  1;
					QR_Flag = 1;
				}
//				}
				// ��ջ���??
				memset(RxBuffer, 0, sizeof(RxBuffer));
				Uart10_Rx_Cnt = 0;
			}
		}
		// ������������
		HAL_UART_Receive_IT(&huart10, (uint8_t*) &aRxBuffer, 1);
	}
//	else if (huart->Instance == USART2)                            // �ж��ж�
//	{
//
//		static uint8_t rx_buffer[11];
//		static uint8_t rx_index = 0;
//		static uint8_t state = 0; // 0: Ѱ�Ұ�ͷ???1: ��������
//
//		if (state == 0) { // Ѱ�Ұ�ͷ
//
//			if (HwtData == 0x55) {
//				rx_buffer[0] = HwtData;
//				rx_index = 1;
//				state = 1;
//			}
//		} else if (state == 1) { // ��������
//			rx_buffer[rx_index++] = HwtData;
//			if (rx_index == 2 && rx_buffer[1] != 0x53 && rx_buffer[1] != 0x52) { // ����ڶ��ֽڲ���0x53???0x52�������Ѱ����һ???0x55
//				if (rx_buffer[1] == 0x55) {
//					rx_index = 1;
//				} else {
//					state = 0;
//					rx_index = 0;
//				}
//			} else if (rx_index == 11) { // ���յ������������ݰ�
//				ParseAndPrintData(rx_buffer, 11);
//				rx_index = 0;
//				state = 0; // ����Ѱ�Ұ�ͷ
//			}
//		}
//		HAL_UART_Receive_IT(&huart2, &HwtData, 1);
//
//	}
//
//	else if (huart->Instance == USART10)                            // �ж��ж�
//	{
//		Recive_Joydata(Joy_data);                             	// ����Joy_data����
//		HAL_UART_Receive_IT(&huart10, &Joy_data, 1);       // �����жϻص�
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
//	����������������ص�
	else if (htim == &htim12)
	{
		if (Apply_Chock == true)
		{
			if (Base_Data == true)
			{
				Choke_Flag = false;
				Apply_Chock = false;
				HAL_TIM_Base_Stop_IT(&htim12);
				Base_Data = false;
			}
			else
			{
				return;
			}
		}
//		���ݲ�ѯ�����ȷ���Ƿ�Ҫ�������� Choke_Flag
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
