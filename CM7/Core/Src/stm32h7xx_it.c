/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32h7xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "absolute_encoder.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

uint8_t TxData_Node1_To_Node2[64] = { 0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11,
		0x22, 0x33, 0x44, 0x55, 0x66 };
//uint8_t RxData_From_Node2[12];
uint8_t Motor_Stator[1] = { 0x00 };
uint8_t Motor_Operation[2] = { 0x01, 0x00 };
uint8_t Motor_Torque[6] = { 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00 };

int Actuator_flag = 0;
int Motor_flag = 0;
int IRQ_flag = 0;

int decimal = 0;
int decimal2 = 0;
int decimal3 = 0;

uint8_t hexa[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t hexa2[2] = { 0x00, 0x00 };
uint8_t hexa3[2] = { 0x00, 0x00 };
uint8_t hexa_final[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

int ctl_timer = 0;
int operation_timer = 0;
int Rx_start_timer = 0;
int Tx_start_timer = 0;
int second_timer = 0;

int Rx_start = 0;
extern Rx_buffer[24];

float pos_test[3] = { 225, 225, 225 };
float vel_test[3] = { 0, };

int pos_max_limit = 300;
int pos_min_limit = 110;

int angle_3_temp = 225;
int angle_2_temp = 225;
int angle_1_temp = 225;

extern int mode_switch;
extern char Tx_buffer[18];

//extern spiAbsoluteEncoder absoluteEn_1;
//extern spiAbsoluteEncoder absoluteEn_2;
//extern spiAbsoluteEncoder absoluteEn_3;

int motor_start = 1;

int tuning_ang_1 = 0;
int tuning_ang_2 = 0;
int tuning_ang_3 = 0;

//char Tx_buffer[18];

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

	/* USER CODE END DMA1_Stream0_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
	/* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

	/* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

	/* USER CODE END DMA1_Stream1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
	/* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

	/* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

	/* USER CODE END DMA1_Stream2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart6_rx);
	/* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

	/* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

	/* USER CODE END DMA1_Stream3_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart6_tx);
	/* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

	/* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
 * @brief This function handles FDCAN1 interrupt 0.
 */
void FDCAN1_IT0_IRQHandler(void) {
	/* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

	IRQ_flag++;

	/* USER CODE END FDCAN1_IT0_IRQn 0 */
	HAL_FDCAN_IRQHandler(&hfdcan1);
	/* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

	HAL_FDCAN_IRQHandler(&hfdcan1);

//	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader,
//			RxData_From_Node2) != HAL_OK) {
//
//	}

	/* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
 * @brief This function handles FDCAN1 interrupt 1.
 */
void FDCAN1_IT1_IRQHandler(void) {
	/* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */
	IRQ_flag++;

	/* USER CODE END FDCAN1_IT1_IRQn 0 */
	HAL_FDCAN_IRQHandler(&hfdcan1);
	/* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

	/* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
void TIM7_IRQHandler(void) {
	/* USER CODE BEGIN TIM7_IRQn 0 */
	// motor control period 0.04ms
	// timer period 0.0001s (0.1ms)
	HAL_TIM_IRQHandler(&htim7);

	ctl_timer++;
	Tx_start_timer++;
	Actuator_flag++;

	if ((Tx_start_timer % 100) == 0) {

		getPosSPI1(&absoluteEn_1);
		m_state_1.angle = absoluteEn_1.spiPos * (360.0 / 4096.0);
		getPosSPI3(&absoluteEn_2);
		m_state_2.angle = absoluteEn_2.spiPos * (360.0 / 4096.0);
		getPosSPI5(&absoluteEn_3);
		m_state_3.angle = absoluteEn_3.spiPos * (360.0 / 4096.0);

		Tx_buffer[0] = 97;
		sprintf(&Tx_buffer[1], "%d", (int) (m_state_1.angle * 100));
		Tx_buffer[6] = 98;
		sprintf(&Tx_buffer[7], "%d", (int) (m_state_2.angle * 100));
		Tx_buffer[12] = 99;
		sprintf(&Tx_buffer[13], "%d", (int) (m_state_3.angle * 100));

		HAL_UART_Transmit_DMA(&huart6, Tx_buffer, strlen(Tx_buffer));
	}

	if (mode_switch == 1) {
		if ((ctl_timer % 40) == 0) {
			angle_1_temp = m_state_1.angle + tuning_ang_1;
			angle_2_temp = m_state_2.angle + tuning_ang_2;
			angle_3_temp = m_state_3.angle + tuning_ang_3;

			//Position_Controller
			if (imp_1.F_flag == 1 || imp_2.F_flag == 1 || imp_3.F_flag == 1)
			{
				pos_ctl_1.goal = imp_1.theta + tuning_ang_1;
				pos_ctl_2.goal = imp_2.theta + tuning_ang_2;
				pos_ctl_3.goal = imp_3.theta + tuning_ang_3;
			}
//			else if (imp_1.F_flag == 0 && imp_2.F_flag == 0 && imp_3.F_flag == 0)
//			{
//				pos_ctl_1.goal = pos_ctl_1.goal;
//				pos_ctl_2.goal = pos_ctl_2.goal + tuning_ang_2 ;
//				pos_ctl_3.goal = pos_ctl_3.goal;
//			}

//			pos_ctl_1.goal = imp_1.theta;
//			pos_ctl_2.goal = imp_2.theta;
//			pos_ctl_3.goal = imp_3.theta;

			vel_ctl_1.goal = Position_Controller(&pos_ctl_1, &m_state_1,
					&profile_1);
			vel_ctl_2.goal = Position_Controller(&pos_ctl_2, &m_state_2,
					&profile_2);
			vel_ctl_3.goal = Position_Controller(&pos_ctl_3, &m_state_3,
					&profile_3);

			if (pos_ctl_1.goal < pos_min_limit)
				pos_ctl_1.goal = pos_min_limit;
			else if (pos_ctl_1.goal > pos_max_limit)
				pos_ctl_1.goal = pos_max_limit;

			if (pos_ctl_2.goal < pos_min_limit)
				pos_ctl_2.goal = pos_min_limit;
			else if (pos_ctl_2.goal > pos_max_limit)
				pos_ctl_2.goal = pos_max_limit;

			if (pos_ctl_3.goal < pos_min_limit)
				pos_ctl_3.goal = pos_min_limit;
			else if (pos_ctl_3.goal > pos_max_limit)
				pos_ctl_3.goal = pos_max_limit;
		}
		if ((ctl_timer % 4) == 0) {

			//Velocity_Controller
			vel_ctl_1.goal = -(pos_ctl_1.output);
			vel_ctl_2.goal = -(pos_ctl_2.output);
			vel_ctl_3.goal = -(pos_ctl_3.output);

			decimal = Velocity_Controller(&vel_ctl_1, &pos_ctl_1, &m_state_1,
					&profile_1, &imp_1);
			decimal2 = Velocity_Controller(&vel_ctl_2, &pos_ctl_2, &m_state_2,
					&profile_2, &imp_2);
			decimal3 = Velocity_Controller(&vel_ctl_3, &pos_ctl_3, &m_state_3,
					&profile_3, &imp_3);

			// Extract most significant byte and store in array
			hexa3[1] = (decimal3 >> 8) & 0xFF;
			// Extract least significant byte and store in array
			hexa3[0] = decimal3 & 0xFF;
			// Extract most significant byte and store in array
			hexa2[1] = (decimal2 >> 8) & 0xFF;
			// Extract least significant byte and store in array
			hexa2[0] = decimal2 & 0xFF;
			// Extract most significant byte and store in array
			hexa[1] = (decimal >> 8) & 0xFF;
			// Extract least significant byte and store in array
			hexa[0] = decimal & 0xFF;

			hexa_final[0] = hexa[0];
			hexa_final[1] = hexa[1];

			hexa_final[2] = hexa2[0];
			hexa_final[3] = hexa2[1];

			hexa_final[4] = hexa3[0];
			hexa_final[5] = hexa3[1];

			if ((Actuator_flag % 100) == 0 && motor_start == 1) {
				TxHeader.Identifier = 0x00;
				TxHeader.IdType = FDCAN_STANDARD_ID;
				TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				TxHeader.DataLength = FDCAN_DLC_BYTES_2;
				TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				TxHeader.BitRateSwitch = FDCAN_BRS_ON;
				TxHeader.FDFormat = FDCAN_FD_CAN;
				TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				TxHeader.MessageMarker = 0x0; // Ignore because FDCAN_NO_TX_EVENTS
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader,
						Motor_Operation) != HAL_OK) {
					Error_Handler();
				}
				operation_timer++;
				if (operation_timer > 20000) {
					Motor_flag = 1;
					motor_start = 0;
				}
			}

			if (Motor_flag == 1) {
				TxHeader.Identifier = 0x300;
				TxHeader.IdType = FDCAN_STANDARD_ID;
				TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				TxHeader.DataLength = FDCAN_DLC_BYTES_6;
				TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				TxHeader.BitRateSwitch = FDCAN_BRS_ON;
				TxHeader.FDFormat = FDCAN_FD_CAN;
				TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				TxHeader.MessageMarker = 0x0; // Ignore because FDCAN_NO_TX_EVENTS

				//if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, Motor_Torque) != HAL_OK)
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader,
						hexa_final) != HAL_OK) {
					Error_Handler();
				} else

					TxHeader.Identifier = 0x080;
				TxHeader.IdType = FDCAN_STANDARD_ID;
				TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
				TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				TxHeader.BitRateSwitch = FDCAN_BRS_ON;
				TxHeader.FDFormat = FDCAN_FD_CAN;
				TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				TxHeader.MessageMarker = 0x0; // Ignore because FDCAN_NO_TX_EVENTS
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, NULL)
						!= HAL_OK) {
					Error_Handler();
				}

				Rx_start_timer++;
				if (Rx_start_timer > 10000) {
					Rx_start = 1;
				}

				if (Rx_start == 1) {
					HAL_UART_Receive_DMA(&huart6, Rx_buffer, sizeof(Rx_buffer));
					Rx_start = 0;
				}
			}
		}
	} else if (mode_switch == 0) {
		if ((ctl_timer % 40) == 0) {

			//Position_Controller
			pos_ctl_1.goal = angle_1_temp + tuning_ang_1;
			pos_ctl_2.goal = angle_2_temp + tuning_ang_2;
			pos_ctl_3.goal = angle_3_temp + tuning_ang_3;

			vel_ctl_1.goal = Position_Controller(&pos_ctl_1, &m_state_1,
					&profile_1);
			vel_ctl_2.goal = Position_Controller(&pos_ctl_2, &m_state_2,
					&profile_2);
			vel_ctl_3.goal = Position_Controller(&pos_ctl_3, &m_state_3,
					&profile_3);

			if (pos_ctl_1.goal < pos_min_limit)
				pos_ctl_1.goal = pos_min_limit;
			else if (pos_ctl_1.goal > pos_max_limit)
				pos_ctl_1.goal = pos_max_limit;

			if (pos_ctl_2.goal < pos_min_limit)
				pos_ctl_2.goal = pos_min_limit;
			else if (pos_ctl_2.goal > pos_max_limit)
				pos_ctl_2.goal = pos_max_limit;

			if (pos_ctl_3.goal < pos_min_limit)
				pos_ctl_3.goal = pos_min_limit;
			else if (pos_ctl_3.goal > pos_max_limit)
				pos_ctl_3.goal = pos_max_limit;
		}
		if ((ctl_timer % 4) == 0) {

			//Velocity_Controller
			vel_ctl_1.goal = -(pos_ctl_1.output);
			vel_ctl_2.goal = -(pos_ctl_2.output);
			vel_ctl_3.goal = -(pos_ctl_3.output);

//			vel_ctl_1.goal = vel_test[0];
//			vel_ctl_2.goal = vel_test[1];
//			vel_ctl_3.goal = vel_test[2];

			decimal = Velocity_Controller(&vel_ctl_1, &pos_ctl_1, &m_state_1,
					&profile_1, &imp_1);
			decimal2 = Velocity_Controller(&vel_ctl_2, &pos_ctl_2, &m_state_2,
					&profile_2, &imp_2);
			decimal3 = Velocity_Controller(&vel_ctl_3, &pos_ctl_3, &m_state_3,
					&profile_3, &imp_3);

//			decimal  = 0;
//			decimal2 = 0;
//			decimal3 = 0;

			// Extract most significant byte and store in array
			hexa3[1] = (decimal3 >> 8) & 0xFF;
			// Extract least significant byte and store in array
			hexa3[0] = decimal3 & 0xFF;
			// Extract most significant byte and store in array
			hexa2[1] = (decimal2 >> 8) & 0xFF;
			// Extract least significant byte and store in array
			hexa2[0] = decimal2 & 0xFF;
			// Extract most significant byte and store in array
			hexa[1] = (decimal >> 8) & 0xFF;
			// Extract least significant byte and store in array
			hexa[0] = decimal & 0xFF;

			hexa_final[0] = hexa[0];
			hexa_final[1] = hexa[1];

			hexa_final[2] = hexa2[0];
			hexa_final[3] = hexa2[1];

			hexa_final[4] = hexa3[0];
			hexa_final[5] = hexa3[1];

			if ((Actuator_flag % 100) == 0 && motor_start == 1) {
				TxHeader.Identifier = 0x00;
				TxHeader.IdType = FDCAN_STANDARD_ID;
				TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				TxHeader.DataLength = FDCAN_DLC_BYTES_2;
				TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				TxHeader.BitRateSwitch = FDCAN_BRS_ON;
				TxHeader.FDFormat = FDCAN_FD_CAN;
				TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				TxHeader.MessageMarker = 0x0; // Ignore because FDCAN_NO_TX_EVENTS
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader,
						Motor_Operation) != HAL_OK) {
					Error_Handler();
				}
				operation_timer++;
				if (operation_timer > 500) {
					Motor_flag = 1;
					motor_start = 0;
				}
			}

			if (Motor_flag == 1) {
				TxHeader.Identifier = 0x300;
				TxHeader.IdType = FDCAN_STANDARD_ID;
				TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				TxHeader.DataLength = FDCAN_DLC_BYTES_6;
				TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				TxHeader.BitRateSwitch = FDCAN_BRS_ON;
				TxHeader.FDFormat = FDCAN_FD_CAN;
				TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				TxHeader.MessageMarker = 0x0; // Ignore because FDCAN_NO_TX_EVENTS

				//if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, Motor_Torque) != HAL_OK)
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader,
						hexa_final) != HAL_OK) {
					Error_Handler();
				} else

					TxHeader.Identifier = 0x080;
				TxHeader.IdType = FDCAN_STANDARD_ID;
				TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				TxHeader.DataLength = FDCAN_DLC_BYTES_0;
				TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				TxHeader.BitRateSwitch = FDCAN_BRS_ON;
				TxHeader.FDFormat = FDCAN_FD_CAN;
				TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				TxHeader.MessageMarker = 0x0; // Ignore because FDCAN_NO_TX_EVENTS
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, NULL)
						!= HAL_OK) {
					Error_Handler();
				}

				Rx_start_timer++;
				if (Rx_start_timer > 5000) {
					Rx_start = 1;
				}

				if (Rx_start == 1) {
					HAL_UART_Receive_DMA(&huart6, Rx_buffer, sizeof(Rx_buffer));
					Rx_start = 0;
				}
			}
		}

	}

	/* USER CODE END TIM7_IRQn 0 */
	HAL_TIM_IRQHandler(&htim7);
	/* USER CODE BEGIN TIM7_IRQn 1 */

	/* USER CODE END TIM7_IRQn 1 */
}

/**
 * @brief This function handles FDCAN calibration unit interrupt.
 */
void FDCAN_CAL_IRQHandler(void) {
	/* USER CODE BEGIN FDCAN_CAL_IRQn 0 */

	/* USER CODE END FDCAN_CAL_IRQn 0 */
	HAL_FDCAN_IRQHandler(&hfdcan1);
	/* USER CODE BEGIN FDCAN_CAL_IRQn 1 */

	/* USER CODE END FDCAN_CAL_IRQn 1 */
}

/**
 * @brief This function handles USART6 global interrupt.
 */
void USART6_IRQHandler(void) {
	/* USER CODE BEGIN USART6_IRQn 0 */

	/* USER CODE END USART6_IRQn 0 */
	HAL_UART_IRQHandler(&huart6);
	/* USER CODE BEGIN USART6_IRQn 1 */

	/* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
