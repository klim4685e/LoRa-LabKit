#include "stm32wlxx.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "usart.h"
#include "anchor/console/console.h"

#include "sx126x_driver.h"
#include "lwip/tcpip.h"

CONSOLE_COMMAND_DEF(tasklist, "tasklist");
static void tasklist_command_handler(const tasklist_args_t* args)
{
    char buf[255];
    vTaskList(buf);
    _printf(buf);
}
CONSOLE_COMMAND_DEF(heap, "tasklist");
static void heap_command_handler(const heap_args_t* args)
{
    uint32_t heap = xPortGetFreeHeapSize();
    _printf("Free heap size = %d \n", heap);
}
CONSOLE_COMMAND_DEF(subghz_init, "subghz_init");
static void subghz_init_command_handler(const subghz_init_args_t* args)
{
    _printf("SUBGHZ_INIT\n");
    tcpip_init(NULL,NULL);
    sx126x_driver_init();
}
CONSOLE_COMMAND_DEF(subghz_rx, "subghz_rx");
static void subghz_rx_command_handler(const subghz_rx_args_t* args)
{
    _printf("SUBGHZ_SET_RX\n");
    sx126x_driver_set_rx();
}
CONSOLE_COMMAND_DEF(subghz_random, "subghz_random");
static void subghz_random_command_handler(const subghz_random_args_t* args)
{
    _printf("subghz_random\n");

    _printf("%d \n",sx126x_driver_random());
}
CONSOLE_COMMAND_DEF(subghz_send, "subghz_send", CONSOLE_STR_ARG_DEF(payload,"payload"));
static void subghz_send_command_handler(const subghz_send_args_t* args)
{
	_printf("subghz_send\n");
	sx126x_driver_send(args->payload, strlen(args->payload));
}
CONSOLE_COMMAND_DEF(subghz_freq, "subghz_freq");
static void subghz_freq_command_handler(const subghz_freq_args_t* args)
{

}

void Console_Task(void* ctx);

__attribute((noreturn)) int main(void)
{
	SystemCoreClockUpdate();
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE5_1 | GPIO_MODER_MODE4_1);
	GPIOB->MODER |= (GPIO_MODER_MODE4_0);
	GPIOB->BSRR = GPIO_BSRR_BS4;
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
	console_command_register(tasklist);
	console_command_register(heap);
	console_command_register(subghz_init);
	console_command_register(subghz_random);
	console_command_register(subghz_send);
	console_command_register(subghz_rx);
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
