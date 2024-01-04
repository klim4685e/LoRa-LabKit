#include "stm32wle5xx.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
void vMainTask(void* ctx);
void vMainTask1(void* ctx);
__attribute((noreturn)) int main(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE4);
	GPIOB->MODER |= GPIO_MODER_MODE5_0 | GPIO_MODER_MODE4_0;
	GPIOB->BSRR = GPIO_BSRR_BR5;
	while(1)
	{
		xTaskCreate(vMainTask,"biba_boba",configMINIMAL_STACK_SIZE,(void*)NULL,1,(void*)NULL);
		xTaskCreate(vMainTask1,"biba_boba1",configMINIMAL_STACK_SIZE,(void*)NULL,1,(void*)NULL);
		vTaskStartScheduler();
		while(1)
		{};
	}

}
void vMainTask(void* ctx)
{
	while(1)
	{
		GPIOB->BSRR = GPIO_BSRR_BR5;
		vTaskDelay(500);
		GPIOB->BSRR = GPIO_BSRR_BS5;
		vTaskDelay(500);
	}
}
void vMainTask1(void* ctx)
{
	while(1)
	{
		GPIOB->BSRR = GPIO_BSRR_BR4;
		vTaskDelay(250);
		GPIOB->BSRR = GPIO_BSRR_BS4;
		vTaskDelay(250);
	}
}
