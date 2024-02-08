/*
 * usart.h
 *
 *  Created on: 4 янв. 2024 г.
 *      Author: klime
 */

#ifndef SRC_INCLUDE_USART_H_
#define SRC_INCLUDE_USART_H_
#include "stm32wlxx.h"

#pragma pack(push,1)
typedef struct usart_struct
{
	uint16_t length;
	uint8_t rxbuffer[2048];
}usart_struct_t;

typedef enum
{
	USART_RX,
	USART_TX,
	USART_BR
} USART_Command_t;

typedef struct
{
	TaskHandle_t taskID;
	USART_Command_t command;
	uint16_t len;
	uint8_t* data;
} USART_Queue_t;

#pragma pack(pop)


uint8_t USART_Init(uint8_t USARTx_Num, uint32_t bandwidth);
uint16_t USART_Receive(uint8_t USART_Num, uint8_t* data, uint16_t max_len);
uint16_t USART_Transmit(uint8_t USART_Num, uint8_t* data, uint16_t len);
uint16_t printf2(const char *format, ...);
#endif /* SRC_INCLUDE_USART_H_ */
