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
#define MAX_MSG_SIZE 256

#define USART_SIZE 1024

#define RXDMA1_Channel_USART(x)	(DMA1_BASE + (0x00000008UL + (0x00000014UL*(x-1))))
#define TXDMA1_Channel_USART(x) (DMA1_BASE + (0x00000008UL + (0x00000014UL*(x+1))))
#define USART(x)				(PERIPH_BASE + (0x00020000UL - 0x00010000UL*(x)) + 0x00003800UL + (0x00004400UL-0x00003800UL)*(x-1))


uint16_t USART_Tail[2];
QueueHandle_t USART_Queue[2];
TaskHandle_t USART_TaskHandle[2];
SemaphoreHandle_t USART_TxSemaphore[2];
uint8_t USART_Buffer[2][USART_SIZE];

void USART_Main(uint8_t USART_num);


uint16_t USART_Receive(uint8_t USART_Num, uint8_t* data, uint16_t max_len)
{
	uint16_t len = 0;
	USART_Queue_t USART_Request;
	USART_Request.taskID = xTaskGetCurrentTaskHandle();
	USART_Request.command = USART_RX;
	USART_Request.data = data;
	USART_Request.len = max_len;
	USART_Queue_t* USART_Request_ptr = &USART_Request;

	xQueueSend(USART_Queue[USART_Num - 1],&USART_Request_ptr,sizeof(USART_Queue_t*));

	ulTaskNotifyTake(USART_TaskHandle[USART_Num - 1],portMAX_DELAY);

	len = USART_Request.len;
	return len;
}
uint16_t USART_Transmit(uint8_t USART_Num, uint8_t* data, uint16_t len)
{
	USART_Queue_t USART_Request;
	USART_Request.taskID = xTaskGetCurrentTaskHandle();
	USART_Request.command = USART_TX;
	USART_Request.data = data;
	USART_Request.len = len;
	USART_Queue_t* USART_Request_ptr = &USART_Request;
	if(xSemaphoreTake(USART_TxSemaphore[USART_Num - 1],100) == pdTRUE)
	{
		xQueueSend(USART_Queue[USART_Num - 1],&USART_Request_ptr,sizeof(USART_Queue_t*));
	}
	if(USART_Num == 2)
		GPIOB->BSRR = GPIO_BSRR_BS5;
	return USART_Request.len;

}

void USART2_Task(void* ctx)
{
	USART_TaskHandle[1] = xTaskGetCurrentTaskHandle();
	uint8_t USART_num = 2;
	while(1)
	{
		USART_Main(USART_num);
	}
}
void USART1_Task(void* ctx)
{
	USART_TaskHandle[0] = xTaskGetCurrentTaskHandle();
	uint8_t USART_num = 1;
	while(1)
	{
		USART_Main(USART_num);
	}
}
uint8_t USART_Init(uint8_t USART_num, uint32_t bandwidth)
{
	volatile USART_TypeDef* USARTx = (USART_TypeDef*)USART(USART_num);
	if(USARTx == USART1)
	{
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
		GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
		GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
		GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFSEL6_Pos) | (7 << GPIO_AFRL_AFSEL7_Pos);
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
		DMAMUX1_Channel0->CCR = 17;
		DMA1_Channel1->CPAR = &USART1->RDR;
		DMA1_Channel1->CMAR = USART_Buffer[USART_num - 1];
		DMA1_Channel1->CNDTR = sizeof(USART_Buffer[USART_num - 1]);
		DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_EN;

		DMAMUX1_Channel2->CCR = 18;
		DMA1_Channel3->CPAR = &USART1->TDR;
		DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
		NVIC_SetPriority(DMA1_Channel3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
		NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	}
	else if(USARTx == USART2)
	{

		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
		GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1;
		GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL3_Pos) | (7 << GPIO_AFRL_AFSEL2_Pos);
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
		DMAMUX1_Channel1->CCR = 19;
		DMA1_Channel2->CPAR = &USART2->RDR;
		DMA1_Channel2->CMAR = USART_Buffer[USART_num - 1];
		DMA1_Channel2->CNDTR = sizeof(USART_Buffer[USART_num - 1]);
		DMA1_Channel2->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_EN;

		DMAMUX1_Channel3->CCR = 20;
		DMA1_Channel4->CPAR = &USART2->TDR;
		DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
		NVIC_SetPriority(DMA1_Channel4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	}

	float usartdiv = SystemCoreClock/(float)bandwidth;
	if(usartdiv - (uint32_t)usartdiv >= 0.5)
		usartdiv++;
	USARTx->BRR = (uint32_t)usartdiv;

	USARTx->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	USARTx->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;

	USART_Tail[USART_num - 1] = 0;
	USART_Queue[USART_num - 1] = xQueueCreate(5,sizeof(uint32_t*));
	USART_TxSemaphore[USART_num - 1] = xSemaphoreCreateBinary();
	xSemaphoreGive(USART_TxSemaphore[USART_num - 1]);
	if(USARTx == USART1)
	{
		xTaskCreate(USART1_Task,"USART1_Task",configMINIMAL_STACK_SIZE*2,(void*)NULL,3,(void*)NULL);
	}

	else if(USARTx == USART2)
	{
		xTaskCreate(USART2_Task,"USART2_Task",configMINIMAL_STACK_SIZE*3,(void*)NULL,4,(void*)NULL);
	}
	return 0;
}

void DMA1_Channel4_IRQHandler(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	GPIOB->BSRR = GPIO_BSRR_BR5;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(USART_TxSemaphore[1],&xHigherPriorityTaskWoken);
	USART2->ICR |= USART_ICR_TCCF;
	DMA1->IFCR |= DMA_IFCR_CGIF4;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void DMA1_Channel3_IRQHandler(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(USART_TxSemaphore[0],&xHigherPriorityTaskWoken);
	USART1->ICR |= USART_ICR_TCCF;
	DMA1->IFCR |= DMA_IFCR_CGIF3;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void USART_Main(uint8_t USART_num)
{
	USART_Queue_t* USART_Request;
	volatile DMA_Channel_TypeDef* USART_RXDMA_Channel = RXDMA1_Channel_USART(USART_num);
	volatile DMA_Channel_TypeDef* USART_TXDMA_Channel = TXDMA1_Channel_USART(USART_num);
	if(xQueueReceive(USART_Queue[USART_num-1],&USART_Request,portMAX_DELAY) == pdTRUE)
	{

		switch(USART_Request->command)
		{
		case USART_RX:

			if(USART_SIZE - USART_RXDMA_Channel->CNDTR >= USART_Tail[USART_num-1])
			{
				uint16_t avail_len = USART_SIZE - USART_RXDMA_Channel->CNDTR - USART_Tail[USART_num-1];

				if(avail_len == 0)
					{
					USART_Request->len = avail_len;
					xTaskNotifyGive(USART_Request->taskID);
					}

				else if(USART_Request->len >= avail_len)
				{
					memcpy(USART_Request->data,&USART_Buffer[USART_num-1][USART_Tail[USART_num-1]],avail_len);
					USART_Tail[USART_num-1] += avail_len;
					USART_Request->len = avail_len;
					xTaskNotifyGive(USART_Request->taskID);
				}
				else
				{
					memcpy(USART_Request->data,&USART_Buffer[USART_num-1][USART_Tail[USART_num-1]],USART_Request->len);
					USART_Tail[USART_num-1] += USART_Request->len;
					xTaskNotifyGive(USART_Request->taskID);
				}
			}
			else
			{
				uint16_t avail_len = (USART_SIZE - USART_RXDMA_Channel->CNDTR) + (USART_SIZE - USART_Tail[USART_num-1]);
				if(USART_Request->len >= avail_len)
				{
					uint16_t temp_len = avail_len;
					memcpy(USART_Request->data,&USART_Buffer[USART_num-1][USART_Tail[USART_num-1]],USART_SIZE - USART_Tail[USART_num-1]);
					temp_len -= USART_SIZE - USART_Tail[USART_num-1];
					memcpy(&USART_Request->data[USART_SIZE - USART_Tail[USART_num-1]],&USART_Buffer[USART_num-1][0],temp_len);
					USART_Tail[USART_num-1] = temp_len;
					USART_Request->len = avail_len;
					xTaskNotifyGive(USART_Request->taskID);
				}
				else
				{
					uint16_t temp_len = USART_Request->len;
					memcpy(USART_Request->data,&USART_Buffer[USART_num-1][USART_Tail[USART_num-1]],USART_SIZE - USART_Tail[USART_num-1]);
					temp_len -= USART_SIZE - USART_Tail[USART_num-1];
					memcpy(&USART_Request->data[USART_SIZE - USART_Tail[USART_num-1]],&USART_Buffer[USART_num-1][0],temp_len);
					USART_Tail[USART_num-1] = temp_len;
					xTaskNotifyGive(USART_Request->taskID);
				}
			}
			break;

		case USART_TX:
			USART_TXDMA_Channel->CCR &= ~DMA_CCR_EN;
			USART_TXDMA_Channel->CMAR = USART_Request->data;
			USART_TXDMA_Channel->CNDTR = USART_Request->len;
			USART_TXDMA_Channel->CCR |= DMA_CCR_EN;
			break;
		}

	}
}
char printf_buffer[MAX_MSG_SIZE];
uint16_t printf2(const char *format, ...)
{
	uint16_t len = 0;
	va_list args;

	if(xSemaphoreTake(USART_TxSemaphore[1 - 1],100) == pdTRUE)
	{
		memset(printf_buffer, 0, MAX_MSG_SIZE);
		xSemaphoreGive(USART_TxSemaphore[1 - 1]);

		va_start(args,format);
		while((*format != '\0') && (len < MAX_MSG_SIZE))
		{
			if(*format != '%')
			{
				len++;
				if(len < MAX_MSG_SIZE)
					strncat(printf_buffer,format,1);
				format++;
			}
			else
			{
				switch(*++format)
				{
				case 'd':
				{
					int val = va_arg(args,int);
					char temp[11];
					itoa(val,temp,10);
					uint8_t temp_len = strlen(temp);
					len += temp_len;
					if(len < MAX_MSG_SIZE)
						strcat(printf_buffer,temp);
					break;
				}
				case 'c':
				{
					char val = va_arg(args,int);
					len++;
					if(len < MAX_MSG_SIZE)
						strncat(printf_buffer,&val,1);
					break;
				}
				case 's':
				{
					char* val = va_arg(args,char*);
					uint8_t temp_len = strlen(val);
					len += temp_len;
					if(len < MAX_MSG_SIZE)
						strcat(printf_buffer,val);
					break;
				}
				case 'X':
				{
					int val = va_arg(args,int);
					char temp[3];
					len += 3;
					sprintf(temp,"%02X",val);
					if(len < MAX_MSG_SIZE)
						strcat(printf_buffer,temp);
					break;
				}

				default:
					len++;
					if(len < MAX_MSG_SIZE)
						strncat(printf_buffer,&format,1);
					break;
				}
				format++;
			}

		}

		va_end(args);
		USART_Transmit(1,printf_buffer,len);
	}
		return len;

}
