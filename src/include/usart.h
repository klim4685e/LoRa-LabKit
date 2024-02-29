/*
 * usart.h
 *
 *  Created on: 4 янв. 2024 г.
 *      Author: klime
 */

#ifndef SRC_INCLUDE_USART_H_
#define SRC_INCLUDE_USART_H_
#include "stm32wlxx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define PERIPH_USART1
#define PERIPH_USART2
#define USART_RX_LEN	250
#define USART_TX_LEN	250

#pragma pack(push,1)

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

typedef struct
{
	USART_TypeDef* base;
	SemaphoreHandle_t semaphore;
	QueueHandle_t     queue;
	uint16_t tail;
	uint8_t rx_buffer[USART_RX_LEN];
	uint8_t tx_buffer[USART_TX_LEN];
	uint8_t init_status;
	DMA_Channel_TypeDef* dma_rx;
	DMA_Channel_TypeDef* dma_tx;
}usart_t;

typedef struct
{
	USART_TypeDef* base;
	GPIO_TypeDef* gpio;
	uint8_t rx_pin;
	uint8_t tx_pin;
	DMA_TypeDef* dma;
	DMA_Channel_TypeDef* dma_channel_rx;
	DMA_Channel_TypeDef* dma_channel_tx;
	IRQn_Type			dma_rx_irq;
	IRQn_Type			dma_tx_irq;
	uint32_t baudrate;
}usart_config_t;
#pragma pack(pop)


usart_t* USART_Init(usart_config_t* config);
uint16_t USART_Receive(usart_t* dev, uint8_t* data, uint16_t max_len);
uint16_t USART_Transmit(usart_t* dev, uint8_t* data, uint16_t len);
usart_t* USART_GetDev(USART_TypeDef* USARTx);
uint16_t printf2(const char *format, ...);
#endif /* SRC_INCLUDE_USART_H_ */
