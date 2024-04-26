#include "stm32wlxx.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "usart.h"
#include "anchor/console/console.h"

#include "sx126x.h"
#include "sx126x_driver.h"

extern void subghzspi_debug_setup(void);
extern sx126x_t* sx126x_dev;

const sx126x_pa_cfg_params_t hpa_cfg = {
    .pa_duty_cycle = 0x02,
    .hp_max = 0x02,
    .device_sel = 0x00,
    .pa_lut = 0x01
};

CONSOLE_COMMAND_DEF(subghz_init, "subghz_init");
static void subghz_init_command_handler(const subghz_init_args_t* args)
{
    _printf("SUBGHZ_INIT\n");
    subghzspi_debug_setup();
    sx126x_set_standby(sx126x_dev, SX126X_STANDBY_CFG_RC);
    sx126x_set_pkt_type(sx126x_dev, SX126X_PKT_TYPE_LORA);
    sx126x_set_buffer_base_address(sx126x_dev,0x00,0x80);
    sx126x_set_rf_freq(sx126x_dev, 868100000);
    sx126x_set_pa_cfg(sx126x_dev,&hpa_cfg);
    sx126x_set_tx_params(sx126x_dev,5,SX126X_RAMP_10_US);
    sx126x_dev->mod_params.bw = SX126X_LORA_BW_125;
    sx126x_dev->mod_params.cr = SX126X_LORA_CR_4_5;
    sx126x_dev->mod_params.sf = SX126X_LORA_SF7;
    sx126x_dev->mod_params.ldro = 0;

    sx126x_set_lora_mod_params(sx126x_dev,&sx126x_dev->mod_params);
}
CONSOLE_COMMAND_DEF(subghz_random, "subghz_random");
static void subghz_random_command_handler(const subghz_random_args_t* args)
{
    _printf("subghz_random\n");
    uint32_t random;
    sx126x_get_random_numbers(sx126x_dev,&random,1);
    _printf("%d ",random);
}
CONSOLE_COMMAND_DEF(subghz_send, "subghz_send", CONSOLE_STR_ARG_DEF(payload,"payload"));
static void subghz_send_command_handler(const subghz_send_args_t* args)
{

	sx126x_write_buffer(sx126x_dev,0x00,args->payload,strlen(args->payload));
	sx126x_dev->pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
	sx126x_dev->pkt_params.crc_is_on = true;
	sx126x_dev->pkt_params.invert_iq_is_on = false;
	sx126x_dev->pkt_params.pld_len_in_bytes = strlen(args->payload);
	sx126x_dev->pkt_params.preamble_len_in_symb = 8;
	sx126x_set_lora_pkt_params(sx126x_dev,&sx126x_dev->pkt_params);
	sx126x_dev->params->set_rf_mode(sx126x_dev,SX126X_RF_MODE_TX_HPA);
	sx126x_set_tx(sx126x_dev, 0);
}
CONSOLE_COMMAND_DEF(subghz_freq, "subghz_freq");
static void subghz_freq_command_handler(const subghz_freq_args_t* args)
{

}
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
	GPIOB->MODER &= ~(GPIO_MODER_MODE5_1 | GPIO_MODER_MODE4_1);
	GPIOB->MODER |= (GPIO_MODER_MODE4_0);
	GPIOB->BSRR = GPIO_BSRR_BS4;
	usart_config_t config =
	{
	  USART2,
	  GPIOA,
	  3,
	  2,
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
	console_command_register(subghz_init);
	console_command_register(subghz_random);
	console_command_register(subghz_send);
	usart_t* dev = USART_GetDev(USART2);
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
