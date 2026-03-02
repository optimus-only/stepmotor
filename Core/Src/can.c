/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "motor_control.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

void CAN_FilterInit()
{
	CAN_FilterTypeDef filter;
filter.FilterBank = 0; // use filter 0
filter.FilterMode = CAN_FILTERMODE_IDMASK; 
filter.FilterScale = CAN_FILTERSCALE_32BIT; 
filter.FilterIdHigh = 0x0000; //ID HIGH
filter.FilterIdLow = 0x0000; // ID LOW
filter.FilterMaskIdHigh = 0x0000; // MASK HIGH
filter.FilterMaskIdLow=0x000; //MASK LOW
filter.FilterFIFOAssignment = CAN_RX_FIFO0; // use FIFO0
filter.FilterActivation = ENABLE; // 

if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
    Error_Handler();
}
  
}


void CAN_SendMessage(uint32_t id, uint8_t* data, uint8_t len) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = id;        // ??ID
    tx_header.RTR = CAN_RTR_DATA; // ???
    tx_header.IDE = CAN_ID_STD;   // ???
    tx_header.DLC = len;         // ????(0-8??)
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
        
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        switch (rxHeader.StdId) 
        {
            case 0x10: // (Position Mode)
            {
                
                int32_t goal_pos = (int32_t)(rxData[0] | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24)); 
                motor_control.stall_flag = false;
                Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);
                Motor_Control_Write_Goal_Location(goal_pos);
							  motor_control.mode_run = Motor_Mode_Digital_Location	;
								
							
                break;
            }

            case 0x11: //(Speed Mode)
            {
                int32_t goal_spd = (int32_t)(rxData[0] | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24));             
//              Motor_Control_SetMotorMode(Motor_Mode_Debug_Speed); 
//              Motor_Control_Write_Goal_Speed(goal_spd);
                break;
            }

            case 0x12: 
            {
                if(rxData[0] == 0) 
                {
                    Motor_Control_SetMotorMode(Control_Mode_Stop);
                }
                else if(rxData[0] == 1) 
                {
                    
                    Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);
                    Motor_Control_Write_Goal_Location(motor_control.real_location);
                }
                break;
            }
            
            
        }
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
