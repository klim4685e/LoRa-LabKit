/*
 * usart.c
 *
 *  Created on: 4 янв. 2024 г.
 *      Author: klime
 */

#include "stm32wle5xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "usart.h"

#include <stdarg.h>
#include "stdlib.h"
#include "string.h"

#define MAX_MSG_SIZE 255
void USART_Main(usart_t* dev);

#ifdef	PERIPH_USART1
void USART1_Task(void* ctx);
usart_t usart1;
#endif

#ifdef	PERIPH_USART2
void USART2_Task(void* ctx);
usart_t usart2;
#endif

uint16_t USART_Receive(usart_t* dev, uint8_t* data, uint16_t max_len)
{
	uint16_t len = 0;
	USART_Queue_t USART_Request;
	USART_Request.taskID = xTaskGetCurrentTaskHandle();
	USART_Request.command = USART_RX;
	USART_Request.data = data;
	USART_Request.len = max_len;
	USART_Queue_t* USART_Request_ptr = &USART_Request;

	xQueueSend(dev->queue,&USART_Request_ptr,sizeof(USART_Queue_t*));

	ulTaskNotifyTake(0,100);

	len = USART_Request.len;
	return len;
}
uint16_t USART_Transmit(usart_t* dev, uint8_t* data, uint16_t len)
{
	USART_Queue_t USART_Request;
	USART_Request.taskID = xTaskGetCurrentTaskHandle();
	USART_Request.command = USART_TX;
	USART_Request.data = data;
	USART_Request.len = len;
	USART_Queue_t* USART_Request_ptr = &USART_Request;
	if(dev->base == USART2)
		GPIOB->BSRR = GPIO_BSRR_BS5;
	xQueueSend(dev->queue,&USART_Request_ptr,sizeof(USART_Queue_t*));
	ulTaskNotifyTake(0,100);
	if(dev->base == USART2)
		GPIOB->BSRR = GPIO_BSRR_BR5;
	return USART_Request.len;

}
#ifdef PERIPH_USART1
void USART1_Task(void* ctx)
{
	usart_t* dev = &usart1;
	USART_Main(dev);

}
#endif
#ifdef PERIPH_USART2
void USART2_Task(void* ctx)
{
	usart_t* dev = &usart2;
	USART_Main(dev);

}
#endif

usart_t* USART_Init(usart_config_t* config)
{
	usart_t* res = 0;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

	if(config->base == USART1)
	{
		res = &usart1;
		if(res->init_status == 1)
			return res;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		DMAMUX1_Channel0->CCR = 17;
		DMA1_Channel1->CPAR = &USART1->RDR;
		DMA1_Channel1->CMAR = res->rx_buffer;
		DMA1_Channel1->CNDTR = USART_RX_LEN;
		DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_EN;
		res->dma_rx = DMA1_Channel1;
		DMAMUX1_Channel2->CCR = 18;
		DMA1_Channel3->CPAR = &USART1->TDR;
		DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
		NVIC_SetPriority(DMA1_Channel3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
		NVIC_EnableIRQ(DMA1_Channel3_IRQn);
		res->dma_tx = DMA1_Channel3;
	}
	else if(config->base == USART2)
	{
		res = &usart2;
		if(res->init_status == 1)
			return res;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
		DMAMUX1_Channel1->CCR = 19;
		DMA1_Channel2->CPAR = &USART2->RDR;
		DMA1_Channel2->CMAR = res->rx_buffer;
		DMA1_Channel2->CNDTR = USART_RX_LEN;
		DMA1_Channel2->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_EN;
		res->dma_rx = DMA1_Channel2;
		DMAMUX1_Channel3->CCR = 20;
		DMA1_Channel4->CPAR = &USART2->TDR;
		DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
		NVIC_SetPriority(DMA1_Channel4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);
		res->dma_tx = DMA1_Channel4;
	}

	if(config->gpio == GPIOA)
	{
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	}
	else if(config->gpio == GPIOB)
	{
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	}
	else if(config->gpio == GPIOC)
	{
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	}
	config->gpio->MODER &= ~((3 << (config->rx_pin*2)) | (3 << (config->tx_pin*2)));
	config->gpio->MODER |= ((2 << (config->rx_pin*2)) | (2 << (config->tx_pin*2)));

	if(config->rx_pin < 8)
		config->gpio->AFR[0] |= ((7 << config->rx_pin*4)|(7 << config->tx_pin*4));
	else
		config->gpio->AFR[1] |= ((7 << (config->rx_pin - 8)*4)|(7 << (config->tx_pin - 8)*4));

	float usartdiv = SystemCoreClock/(float)config->baudrate;
	if(usartdiv - (uint32_t)usartdiv >= 0.5)
		usartdiv++;
	res->base = config->base;
	res->base->BRR = (uint32_t)usartdiv;

	res->base->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	res->base->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;

	res->tail = 0;
	res->queue = xQueueCreate(10,sizeof(USART_Queue_t*));
	res->semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(res->semaphore);
	res->init_status = 1;


#ifdef	PERIPH_USART1
	if(res->base == USART1)
	{
		xTaskCreate(USART1_Task,"USART1_Task",configMINIMAL_STACK_SIZE,(void*)NULL,configMAX_PRIORITIES-2,(void*)NULL);
	}
#endif
#ifdef	PERIPH_USART2
	else if(res->base == USART2)
	{
		xTaskCreate(USART2_Task,"USART2_Task",configMINIMAL_STACK_SIZE,(void*)NULL,configMAX_PRIORITIES-2,(void*)NULL);
	}
#endif
	return res;

}

#ifdef	PERIPH_USART2
void DMA1_Channel4_IRQHandler(void)
{
	static BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(usart2.semaphore,&xHigherPriorityTaskWoken);
	USART2->ICR |= USART_ICR_TCCF;
	DMA1->IFCR |= DMA_IFCR_CGIF4;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif
#ifdef	PERIPH_USART1
void DMA1_Channel3_IRQHandler(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(usart1.semaphore,&xHigherPriorityTaskWoken);
	USART1->ICR |= USART_ICR_TCCF;
	DMA1->IFCR |= DMA_IFCR_CGIF3;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif
void USART_Main(usart_t* dev)
{
	USART_Queue_t* USART_Request;
	while(1)
	{
	if(xQueueReceive(dev->queue,&USART_Request,portMAX_DELAY) == pdTRUE)
	{
		switch(USART_Request->command)
		{
		case USART_RX:

			if(USART_RX_LEN - dev->dma_rx->CNDTR >= dev->tail)
			{
				uint16_t avail_len = USART_RX_LEN - dev->dma_rx->CNDTR - dev->tail;

				if(avail_len == 0)
					{
					USART_Request->len = avail_len;
					xTaskNotifyGive(USART_Request->taskID);
					break;
					}

				else if(USART_Request->len >= avail_len)
				{
					memcpy(USART_Request->data,&dev->rx_buffer[dev->tail],avail_len);
					dev->tail += avail_len;
					USART_Request->len = avail_len;
					xTaskNotifyGive(USART_Request->taskID);
				}
				else
				{
					memcpy(USART_Request->data,&dev->rx_buffer[dev->tail],USART_Request->len);
					dev->tail += USART_Request->len;
					xTaskNotifyGive(USART_Request->taskID);
				}
			}
			else
			{
				uint16_t avail_len = (USART_RX_LEN - dev->dma_rx->CNDTR) + (USART_RX_LEN - dev->tail);
				if(USART_Request->len >= avail_len)
				{
					uint16_t temp_len = avail_len;
					memcpy(USART_Request->data,&dev->rx_buffer[dev->tail],USART_RX_LEN - dev->tail);
					temp_len -= USART_RX_LEN - dev->tail;
					memcpy(&USART_Request->data[USART_RX_LEN - dev->tail],&dev->rx_buffer[0],temp_len);
					dev->tail = temp_len;
					USART_Request->len = avail_len;
					xTaskNotifyGive(USART_Request->taskID);
				}
				else
				{
					uint16_t temp_len = USART_Request->len;
					memcpy(USART_Request->data,&dev->rx_buffer[dev->tail],USART_RX_LEN - dev->tail);
					temp_len -= USART_RX_LEN - dev->tail;
					memcpy(&USART_Request->data[USART_RX_LEN - dev->tail],&dev->rx_buffer[0],temp_len);
					dev->tail = temp_len;
					xTaskNotifyGive(USART_Request->taskID);
				}
			}
			break;

		case USART_TX:
			if(xSemaphoreTake(dev->semaphore,100) == pdTRUE)
			{
				memcpy(dev->tx_buffer,USART_Request->data,USART_Request->len);
			dev->dma_tx->CCR &= ~DMA_CCR_EN;
			dev->dma_tx->CMAR = (uint32_t)dev->tx_buffer;
			dev->dma_tx->CNDTR = USART_Request->len;
			dev->dma_tx->CCR |= DMA_CCR_EN;
			}
			if(xSemaphoreTake(dev->semaphore,100) == pdTRUE)
			{
				xTaskNotifyGive(USART_Request->taskID);
				xSemaphoreGive(dev->semaphore);
			}
			break;
		}
	}
	}
}

int _printf(const char *format, ...)
{
	uint8_t print_buffer[MAX_MSG_SIZE];
   // Variable Argument List
   va_list arg;

   int done;

   // Get Variable Arguments
   va_start (arg, format);

   // Pass format string and arguments to string formatter
   done = vsnprintf(print_buffer, MAX_MSG_SIZE, format, arg);
   usart_t* dev = USART_GetDev(USART1);
   // Start Transmission
   USART_Transmit(dev,print_buffer,done);

   // End Variable Arguments
   va_end (arg);

   return done;
}
usart_t* USART_GetDev(USART_TypeDef* USARTx)
{
	usart_t* dev = 0;
#ifdef PERIPH_USART1
	if(USARTx == USART1)
	{
		if(usart1.init_status == 1)
		{
			dev = &usart1;
		};
	}
#endif
#ifdef PERIPH_USART2
	if(USARTx == USART2)
	{
		if(usart2.init_status == 1)
		{
			return &usart2;
		};
	}
#endif
	return dev;
}
