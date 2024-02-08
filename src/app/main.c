#include "stm32wle5xx.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "task.h"

#include "pppos_netif.h"
#include "usart.h"
#include "lwip/tcpip.h"


extern QueueHandle_t USART2_Queue;
void vMainTask(void* ctx);
void vMainTask1(void* ctx);



int main(void)
{
	SystemCoreClockUpdate();
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE4);
	GPIOB->MODER |= GPIO_MODER_MODE5_0 | GPIO_MODER_MODE4_0;
	GPIOB->BSRR = GPIO_BSRR_BR5;




	while(1)
	{
		xTaskCreate(vMainTask,"biba_boba",configMINIMAL_STACK_SIZE,(void*)NULL,0,(void*)NULL);
	//	xTaskCreate(vMainTask1,"biba_boba1",configMINIMAL_STACK_SIZE*2,(void*)NULL,0,(void*)NULL);
		vTaskStartScheduler();
		while(1)
		{};
	}
	return 0;

}
void vMainTask(void* ctx)
{
	pppos_init();

	while(1)
	{
		GPIOB->BSRR = GPIO_BSRR_BR4;
		vTaskDelay(500);
		GPIOB->BSRR = GPIO_BSRR_BS4;
		vTaskDelay(500);
	}
}
void vMainTask1(void* ctx)
{
	USART_Init(2,115200);
	USART_Init(1,115200);
	uint8_t control[200];
	uint8_t bebra[] = "bebraBibra\r\n";
			GPIOB->BSRR = GPIO_BSRR_BR4;
	while(1)
	{
		if(USART_Receive(2,control,200) != 0)
			__asm("nop");
		GPIOB->BSRR = GPIO_BSRR_BR4;
		vTaskDelay(250);
		USART_Transmit(2,bebra,sizeof(bebra));
		USART_Transmit(2,bebra,sizeof(bebra));
		USART_Transmit(1,bebra,sizeof(bebra));
		USART_Transmit(1,bebra,sizeof(bebra));
		GPIOB->BSRR = GPIO_BSRR_BS4;
		vTaskDelay(2000);
	}
}
