/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// FDCAN1 Defines
#include "absolute_encoder.h"
#include "motor_control.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//uint8_t TxData_Node1_To_Node2[64] = { 0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11,
//		0x22, 0x33, 0x44, 0x55, 0x66 };
//uint8_t Motor_Stator[1] = { 0x00 };
//uint8_t Motor_Operation[2] = { 0x01, 0x00 };
//uint8_t Motor_Torque[6] = { 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00 };
//uint8_t RxData_From_Node1[64];
//extern spiAbsoluteEncoder absoluteEn_1;
//extern spiAbsoluteEncoder absoluteEn_2;
//extern spiAbsoluteEncoder absoluteEn_3;
//uint8_t hexa2[2] = { 0x00, 0x00 };
//uint8_t hexa3[2] = { 0x00, 0x00 };
//uint8_t hexa_final[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//int Actuator_flag = 0;
//int Motor_flag = 0;
//int imp_torque_Tx_buffer[24] = { 0, };
//int RAMP = 300;
//int Tx_start = 0;
//int Rx_start = 0;
//int Tx_cnt = 0;
//uint8_t motor_state[6] = { 0x00, };
//uint8_t Tx_data[18] = { 0, };
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

Impedance imp_1;
Impedance imp_2;
Impedance imp_3;

uint8_t RxData_From_Node2[12];
uint32_t val_1[3] = { 0, };
uint32_t val_2[3] = { 0, };
uint32_t val_3[3] = { 0, };

uint8_t Motor1[12] = { 0, };
uint8_t Motor2[12] = { 0, };
uint8_t Motor3[12] = { 0, };

int SPI2Test = 0;
int MAINTest = 0;
int Motor_Degree = 0;

extern uint8_t Rx_buffer[24] = { 0, };
int Rx_imp_data[24] = { 0, };
char Tx_buffer[18] = { 0, };

int A_buffer = 0;
int A_flag = 0;
extern int mode_switch = 0;

int vel_ctl_2_kp_tuning = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

	if (HAL_OK
			== HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader,
					RxData_From_Node2)) {
		if (RxHeader.Identifier == 385) {
			Motor1[0] = RxData_From_Node2[0];
			Motor1[1] = RxData_From_Node2[1];
			Motor1[2] = RxData_From_Node2[2];
			Motor1[3] = RxData_From_Node2[3];
			Motor1[4] = RxData_From_Node2[4];
			Motor1[5] = RxData_From_Node2[5];
			Motor1[6] = RxData_From_Node2[6];
			Motor1[7] = RxData_From_Node2[7];
			Motor1[8] = RxData_From_Node2[8];
			Motor1[9] = RxData_From_Node2[9];
			Motor1[10] = RxData_From_Node2[10];
			Motor1[11] = RxData_From_Node2[11];

		} else if (RxHeader.Identifier == 386) {
			Motor2[0] = RxData_From_Node2[0];
			Motor2[1] = RxData_From_Node2[1];
			Motor2[2] = RxData_From_Node2[2];
			Motor2[3] = RxData_From_Node2[3];
			Motor2[4] = RxData_From_Node2[4];
			Motor2[5] = RxData_From_Node2[5];
			Motor2[6] = RxData_From_Node2[6];
			Motor2[7] = RxData_From_Node2[7];
			Motor2[8] = RxData_From_Node2[8];
			Motor2[9] = RxData_From_Node2[9];
			Motor2[10] = RxData_From_Node2[10];
			Motor2[11] = RxData_From_Node2[11];

		} else if (RxHeader.Identifier == 387) {
			Motor3[0] = RxData_From_Node2[0];
			Motor3[1] = RxData_From_Node2[1];
			Motor3[2] = RxData_From_Node2[2];
			Motor3[3] = RxData_From_Node2[3];
			Motor3[4] = RxData_From_Node2[4];
			Motor3[5] = RxData_From_Node2[5];
			Motor3[6] = RxData_From_Node2[6];
			Motor3[7] = RxData_From_Node2[7];
			Motor3[8] = RxData_From_Node2[8];
			Motor3[9] = RxData_From_Node2[9];
			Motor3[10] = RxData_From_Node2[10];
			Motor3[11] = RxData_From_Node2[11];
		}

		val_1[0] = (Motor1[0] << 0) | (Motor1[1] << 8) | (Motor1[2] << 16)
				| (Motor1[3] << 24);
		val_1[1] = (Motor1[4] << 0) | (Motor1[5] << 8) | (Motor1[6] << 16)
				| (Motor1[7] << 24);
		val_1[2] = (Motor1[8] << 0) | (Motor1[9] << 8) | (Motor1[10] << 16)
				| (Motor1[11] << 24);

		val_2[0] = (Motor2[0] << 0) | (Motor2[1] << 8) | (Motor2[2] << 16)
				| (Motor2[3] << 24);
		val_2[1] = (Motor2[4] << 0) | (Motor2[5] << 8) | (Motor2[6] << 16)
				| (Motor2[7] << 24);
		val_2[2] = (Motor2[8] << 0) | (Motor2[9] << 8) | (Motor2[10] << 16)
				| (Motor2[11] << 24);

		val_3[0] = (Motor3[0] << 0) | (Motor3[1] << 8) | (Motor3[2] << 16)
				| (Motor3[3] << 24);
		val_3[1] = (Motor3[4] << 0) | (Motor3[5] << 8) | (Motor3[6] << 16)
				| (Motor3[7] << 24);
		val_3[2] = (Motor3[8] << 0) | (Motor3[9] << 8) | (Motor3[10] << 16)
				| (Motor3[11] << 24);

		//m_state_1.angle = (*(float*) &val_1[0]) * (180.0f / M_PI); //* 180 / PI;
		m_state_1.velocity = *(float*) &val_1[1];
		m_state_1.torque = *(float*) &val_1[2];

		//m_state_2.angle = (*(float*) &val_2[0]) * (180.0f / M_PI); //* 180 / PI;
		m_state_2.velocity = *(float*) &val_2[1];
		m_state_2.torque = *(float*) &val_2[2];

		//m_state_3.angle = (*(float*) &val_3[0]) * (180.0f / M_PI); //* 180 / PI;
		m_state_3.velocity = *(float*) &val_3[1];
		m_state_3.torque = *(float*) &val_3[2];

	}

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

//	for(int i = 0; i < 24 < i==)
	Init_PID();

	/* USER CODE END 1 */
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
		;
	if (timeout < 0) {
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	 HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0, 0);
	/* wait until CPU2 wakes up from stop mode */

	timeout = 0xFFFF;

	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0))
		;
	if (timeout < 0) {
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_SPI4_Init();
	MX_SPI5_Init();
	MX_SPI6_Init();
	MX_USART1_UART_Init();
	MX_TIM7_Init();
	MX_FDCAN1_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim7);

	spiAbsoluteEncoderInitialize(&absoluteEn_1);
	spiAbsoluteEncoderInitialize(&absoluteEn_2);
	spiAbsoluteEncoderInitialize(&absoluteEn_3);

	/* Configure standard ID reception filter to Rx Tx_buffer 0 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterID1 = 0x000; //
	sFilterConfig.FilterID2 = 0xFFF; //

#if 0
  sFilterConfig.FilterID2 = 0x0; // Ignore because FDCAN_FILTER_TO_RXTx_buffer
#endif
	sFilterConfig.RxBufferIndex = 0;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
	FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE)
			== HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
	FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1,
	FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

	/* Send Tx Tx_buffer message */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	/* Polling for transmission complete on Tx_buffer index 0 */

	/* Polling for reception complete on Tx_buffer index 0 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		pos_ctl_1.Kp = pos_test_pid.Kp;
		pos_ctl_1.Ki = pos_test_pid.Ki;
		pos_ctl_1.Kd = pos_test_pid.Kd;
		pos_ctl_1.output_limit = pos_test_pid.output_limit;
		pos_ctl_1.error_sum_limit = pos_test_pid.error_sum_limit;

		pos_ctl_2.Kp = pos_test_pid.Kp;
		pos_ctl_2.Ki = pos_test_pid.Ki;
		pos_ctl_2.Kd = pos_test_pid.Kd;
		pos_ctl_2.output_limit = pos_test_pid.output_limit;
		pos_ctl_2.error_sum_limit = pos_test_pid.error_sum_limit;

		pos_ctl_3.Kp = pos_test_pid.Kp;
		pos_ctl_3.Ki = pos_test_pid.Ki;
		pos_ctl_3.Kd = pos_test_pid.Kd;
		pos_ctl_3.output_limit = pos_test_pid.output_limit;
		pos_ctl_3.error_sum_limit = pos_test_pid.error_sum_limit;

		vel_ctl_1.Kp = vel_test_pid.Kp;
		vel_ctl_1.Ki = vel_test_pid.Ki;
		vel_ctl_1.Kd = vel_test_pid.Kd;
		vel_ctl_1.output_limit = vel_test_pid.output_limit;
		vel_ctl_1.error_sum_limit = vel_test_pid.error_sum_limit;

		vel_ctl_2.Kp = vel_test_pid.Kp + vel_ctl_2_kp_tuning;
		vel_ctl_2.Ki = vel_test_pid.Ki;
		vel_ctl_2.Kd = vel_test_pid.Kd;
		vel_ctl_2.output_limit = vel_test_pid.output_limit;
		vel_ctl_2.error_sum_limit = vel_test_pid.error_sum_limit;

		vel_ctl_3.Kp = vel_test_pid.Kp;
		vel_ctl_3.Ki = vel_test_pid.Ki;
		vel_ctl_3.Kd = vel_test_pid.Kd;
		vel_ctl_3.output_limit = vel_test_pid.output_limit;
		vel_ctl_3.error_sum_limit = vel_test_pid.error_sum_limit;

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 42;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 14;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	for (int i = 0; i < sizeof(Rx_buffer); i++) {
		if (Rx_buffer[i] == 65) {
			A_flag = 1;
			A_buffer = i;
		}
	}

	if (A_flag == 1) {
		for (int j = 0; j < 24; j++) {
			if (A_buffer + j > 23) {
				Rx_imp_data[j] = (int) (int8_t) Rx_buffer[(A_buffer + j) - 24]
						- 48;
			} else {
				Rx_imp_data[j] = (int) (int8_t) Rx_buffer[A_buffer + j] - 48;
			}
		}

		//	// A 65, a 97
		for (int k = 1; k < 2; k++) {
			if (Rx_imp_data[0] == 17) {
				imp_1.theta = Rx_imp_data[k] * (10000)
						+ Rx_imp_data[k + 1] * (1000)
						+ Rx_imp_data[k + 2] * (100) + Rx_imp_data[k + 3] * (10)
						+ Rx_imp_data[k + 4];

				imp_2.theta = Rx_imp_data[k + 6] * (10000)
						+ Rx_imp_data[k + 7] * (1000)
						+ Rx_imp_data[k + 8] * (100) + Rx_imp_data[k + 9] * (10)
						+ Rx_imp_data[k + 10];

				imp_3.theta = Rx_imp_data[k + 12] * (10000)
						+ Rx_imp_data[k + 13] * (1000)
						+ Rx_imp_data[k + 14] * (100)
						+ Rx_imp_data[k + 15] * (10) + Rx_imp_data[k + 16];

				imp_1.F_flag = Rx_imp_data[k + 18];
				imp_2.F_flag = Rx_imp_data[k + 19];
				imp_3.F_flag = Rx_imp_data[k + 20];

				mode_switch = Rx_imp_data[k + 22];

				imp_1.theta = imp_1.theta / 100;
				imp_2.theta = imp_2.theta / 100;
				imp_3.theta = imp_3.theta / 100;
			} else {

			}
		}
		A_flag = 0;
	} else if (A_flag == 0) {
	}

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
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
