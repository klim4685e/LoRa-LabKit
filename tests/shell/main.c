#include "stm32wlxx.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "usart.h"
#include "anchor/console/console.h"

CONSOLE_COMMAND_DEF(sum, "a + b",
    CONSOLE_INT_ARG_DEF(a, "a"),
    CONSOLE_INT_ARG_DEF(b, "b")
);
static void sum_command_handler(const sum_args_t* args)
{
    _printf("%d + %d = %d\n",args->a,args->b,args->a + args->b);
}
void Console_Task(void* ctx);

__attribute((noreturn)) int main(void)
{
	SystemCoreClockUpdate();
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE4);
	GPIOB->MODER |= GPIO_MODER_MODE5_0 | GPIO_MODER_MODE4_0;
	GPIOB->BSRR = GPIO_BSRR_BR5;
	usart_config_t config =
	{
	  USART1,
	  GPIOB,
	  7,
	  6,
	  DMA1,
	  0,
	  0,
	  0,
	  0,
	  115200,
	};
	usart_t* dev = USART_Init(&config);

	while(1)
	{
		xTaskCreate(Console_Task,"biba_boba",configMINIMAL_STACK_SIZE*3,(void*)NULL,1,(void*)NULL);
		vTaskStartScheduler();
		while(1)
		{};
	}

}
void Console_Task(void* ctx)
{
	uint8_t len = 0;
	uint8_t offset = 0;
	uint8_t shellInputString[64];
	const console_init_t init_console = {
		.write_function = _printf,
	  };
	console_init(&init_console);

	console_command_register(sum);
	usart_t* dev = USART_GetDev(USART1);
	while(1)
	{
		len = USART_Receive(dev,&shellInputString[offset],64 - offset);
		if(len > 0)
		{
			for(uint8_t i = 0; i < len;i++)
			{
				_printf("%c",shellInputString[offset + i]);
				if((shellInputString[offset + i] == '\r') || (shellInputString[offset + i] == '\n'))
				{
					shellInputString[offset + i] = '\n';
					console_process(shellInputString,offset + i + 1);
					offset = 0;
					len = 0;
				}
			}
			offset += len;
			if(offset >= 64)
				offset = 0;
		}
		vTaskDelay(5);
	}
}
