/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <stdio.h>
#include "ringbuffer.h"
#ifdef USE_USB_SERIAL
#include "shell.h"
#endif
#ifdef USE_USB_CAN
#include "Can.h"
#include "CanIf_Cbk.h"
#endif
#include "asdebug.h"
#include "Dio.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define AS_LOG_USB 1
#ifndef CAN1_Q_SIZE
#define CAN1_Q_SIZE 32
#endif

#define LED_CANRTX   DIO_CHL_LED3
#define LED_CANERROR DIO_CHL_LED2
/* Private variables ---------------------------------------------------------*/
#ifdef USE_USB_SERIAL
RB_DECLARE(usbio, char, 1024);
#endif
#ifdef USE_USB_CAN
extern int mx_can_write(uint32_t canid, uint8_t dlc, uint8_t* data);
RB_DECLARE(can1in,  Can_SerialInPduType, CAN1_Q_SIZE);
RB_EXTERN(canin);
RB_EXTERN(canout);
#endif
static uint8_t bOnline = FALSE;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  ASLOG(USB,"offline\n");
  bOnline = FALSE;
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
        if(((uint32_t*)pbuf)[0] > 0)
        {
            bOnline = TRUE;
            ASLOG(USB,"online\n");
        }
    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
#ifdef USE_USB_SERIAL
  uint32_t i;
  for(i=0; i<*Len; i++) {
    if('\r' == Buf[i]) {
      Buf[i] = '\n';
    }
    SHELL_input(Buf[i]);
  }
#endif
#ifdef USE_USB_CAN
  rb_size_t r;
  imask_t imask;
  Can_SerialInPduType* pdu = (Can_SerialInPduType*)Buf;
#ifndef __AS_BOOTLOADER__
  if(1 == pdu->busid)
  {
	Irq_Save(imask);
	r = RB_PUSH(can1in, Buf, *Len);
	Irq_Restore(imask);
	if(0 == r)
	{
	  ASLOG(USB,"can1in is full\n");
	}
  }
  else
#endif
  {
    Irq_Save(imask);
    r = RB_PUSH(canin, Buf, *Len);
    Irq_Restore(imask);
    if(0 == r)
    {
      ASLOG(USB,"canin is full\n");
    }
  }
#endif
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void CDC_MainFunction(void)
{
	imask_t imask;
  if(bOnline) {
#ifdef USE_USB_SERIAL
	uint8_t usbTxBuffer[32];
	rb_size_t r;
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

	if(0 == hcdc->TxState) {
		Irq_Save(imask);
		r = RB_Pop(&rb_usbio, usbTxBuffer, sizeof(usbTxBuffer));
		Irq_Restore(imask);
		if(r > 0) {
			CDC_Transmit_FS(usbTxBuffer, r);
		}
	}
#endif

#ifdef USE_USB_CAN
	rb_size_t r;
	uint8_t rv;
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

	if(0 == hcdc->TxState) {
		Can_SerialOutPduType pdu;
		Irq_Save(imask);
		r = RB_POLL(canout, &pdu, sizeof(pdu));
		Irq_Restore(imask);
		if(r > 0) {
			rv = CDC_Transmit_FS((uint8_t*)&pdu, r-sizeof(PduIdType));
			if(USBD_OK == rv)
			{
				Irq_Save(imask);
				RB_DROP(canout, sizeof(pdu));
				Irq_Restore(imask);
				if(((PduIdType)-1) != pdu.swPduHandle)
				{
					CanIf_TxConfirmation(pdu.swPduHandle);
				}
#ifndef __AS_BOOTLOADER__
				else
				{
					Dio_WriteChannel(LED_CANRTX,STD_LOW);
				}
#endif
			}
		}
	}
#ifndef __AS_BOOTLOADER__
  {
	Can_SerialInPduType pdu;
	Irq_Save(imask);
	r = RB_POLL(can1in, &pdu, sizeof(pdu));
	Irq_Restore(imask);
	if(r > 0)
	{
		if(0 == mx_can_write(SCANID(pdu.canid), pdu.dlc, pdu.data))
		{
			Dio_WriteChannel(LED_CANRTX,STD_HIGH);
			Irq_Save(imask);
			RB_DROP(can1in, sizeof(pdu));
			Irq_Restore(imask);
		}
	}
  }
#endif
#endif
  }
}

#ifdef USE_USB_SERIAL
void USB_SerialPutChar(char ch)
{
	imask_t imask;
	Irq_Save(imask);
	RB_Push(&rb_usbio, &ch, 1);
	Irq_Restore(imask);
}
#endif

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	printf("CAN1 error: 0x%X\n", hcan->ErrorCode);
	Dio_WriteChannel(LED_CANERROR,STD_HIGH);

}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	CanRxMsgTypeDef *pRxMsg = NULL;
	Can_SerialOutPduType pdu;
	imask_t imask;
	rb_size_t r;

	if(hcan->pRxMsg->DLC != 0xFF)
	{
		pRxMsg = hcan->pRxMsg;
	}
	else if(hcan->pRxMsg->DLC != 0xFF)
	{
		pRxMsg = hcan->pRx1Msg;
	}
	else
	{
		/*do nothing */
	}

	if(pRxMsg != NULL)
	{
		pdu.busid = 1;

		if(CAN_ID_EXT == pRxMsg->IDE)
		{
			SETSCANID(pdu.canid, 0x80000000 | pRxMsg->ExtId);
		}
		else
		{
			SETSCANID(pdu.canid, pRxMsg->StdId);
		}

		pdu.dlc = pRxMsg->DLC;

		memcpy(pdu.data, pRxMsg->Data, pRxMsg->DLC);

		pdu.swPduHandle = (PduIdType)-1;

		Irq_Save(imask);
		r = RB_PUSH(canout, &pdu, sizeof(pdu));
		Irq_Restore(imask);
		if(0 == r)
		{
			ASLOG(USB, "canout is full\n");
		}

		pRxMsg->DLC = 0xFF;

		Dio_WriteChannel(LED_CANRTX,STD_HIGH);
		Dio_WriteChannel(LED_CANERROR,STD_LOW);
	}

	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
}


void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	UNUSED(hcan);

	Dio_WriteChannel(LED_CANRTX,STD_LOW);
	Dio_WriteChannel(LED_CANERROR,STD_LOW);
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
