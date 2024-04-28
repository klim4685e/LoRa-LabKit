/*
 * sx126x_driver.c
 *
 *  Created on: 26 апр. 2024 г.
 *      Author: hocok
 */

#include "sx126x_driver.h"
#include "sx126x.h"
#include "sx126x_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/pbuf.h"

sx126x_t SX126X_DEV;
sx126x_t* sx126x_dev = &SX126X_DEV;
extern sx126x_hal_status_t sx126x_hal_init(const void* context);
void sx126x_isr_handler(void* ctx);

const sx126x_pa_cfg_params_t hpa_cfg = {
    .pa_duty_cycle = 0x02,
    .hp_max = 0x02,
    .device_sel = 0x00,
    .pa_lut = 0x01
};

void SUBGHZ_Radio_IRQHandler(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
	if((sx126x_dev != NULL) || (sx126x_dev->params->event_handler != NULL))
		vTaskNotifyGiveFromISR(sx126x_dev->params->event_handler,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void sx126x_driver_init(void)
{
	TaskHandle_t created_task = NULL;

	sx126x_hal_init(sx126x_dev);

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

	sx126x_dev->pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
	sx126x_dev->pkt_params.crc_is_on = true;
	sx126x_dev->pkt_params.invert_iq_is_on = false;
	sx126x_dev->pkt_params.preamble_len_in_symb = 8;
	sx126x_set_lora_pkt_params(sx126x_dev,&sx126x_dev->pkt_params);
    const uint16_t irq_mask = (
        SX126X_IRQ_TX_DONE |
        SX126X_IRQ_RX_DONE |
        SX126X_IRQ_PREAMBLE_DETECTED |
        SX126X_IRQ_SYNC_WORD_VALID |
        SX126X_IRQ_HEADER_VALID |
        SX126X_IRQ_HEADER_ERROR |
        SX126X_IRQ_CRC_ERROR |
        SX126X_IRQ_CAD_DONE |
        SX126X_IRQ_CAD_DETECTED |
        SX126X_IRQ_TIMEOUT
        );

    sx126x_set_dio_irq_params(sx126x_dev, irq_mask, irq_mask, 0, 0);
	xTaskCreate(sx126x_isr_handler, "sx126x_isr",configMINIMAL_STACK_SIZE*3,NULL,configMAX_PRIORITIES - 2,&created_task);
	if(created_task == NULL)
		return;
	sx126x_dev->params->event_handler = created_task;
	NVIC_SetPriority(SUBGHZ_Radio_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}
uint32_t sx126x_driver_random(void)
{
    uint32_t random;
    sx126x_get_random_numbers(sx126x_dev,&random,1);
    return random;
}
void sx126x_driver_send(uint8_t* data, uint8_t len)
{
	if(len > 255)
		return;
	sx126x_write_buffer(sx126x_dev,0x00,data,len);

	sx126x_dev->pkt_params.pld_len_in_bytes = len;
	sx126x_set_lora_pkt_params(sx126x_dev,&sx126x_dev->pkt_params);
	sx126x_dev->params->set_rf_mode(sx126x_dev,SX126X_RF_MODE_TX_HPA);
	sx126x_set_tx(sx126x_dev, 0);
}
void sx126x_driver_set_rx(void)
{
	sx126x_dev->params->set_rf_mode(sx126x_dev,SX126X_RF_MODE_RX);
	sx126x_set_rx(sx126x_dev, 0);
}
void sx126x_isr_handler(void* ctx)
{
	while(1)
	{
		if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE)
		{
			sx126x_irq_mask_t irq_mask;
			sx126x_get_and_clear_irq_status(sx126x_dev,&irq_mask);

		    if (irq_mask & SX126X_IRQ_TX_DONE)
		    {
		    	_printf("[sx126x]: SX126X_IRQ_TX_DONE\n");
		    }
		    else if (irq_mask & SX126X_IRQ_RX_DONE)
		    {
		    	uint8_t buffer[128];
		    	_printf("[sx126x]: SX126X_IRQ_RX_DONE\n");
		    	uint8_t len = sx126x_driver_receive(buffer, 128);
		    	struct pbuf* p = pbuf_alloc(PBUF_RAW,len,PBUF_POOL);
		    	if(p != NULL)
		    	{
		    		pbuf_take(p,buffer,len);
		    	}
		    	if(p != NULL)
		    	{
			    	_printf("Received %d bytes = \n",p->len);
			    	uint8_t temp_buffer[127];
			    	pbuf_copy_partial(p,temp_buffer,len,0);
			    	for(uint8_t i = 0; i < len; i++)
			    	{
			    		_printf("%02X ",temp_buffer[i]);
			    	}
			    	_printf("\n");
			    	pbuf_free(p);
		    	}
		    }
		    else if (irq_mask & SX126X_IRQ_PREAMBLE_DETECTED)
		    {
		    }
		    else if (irq_mask & SX126X_IRQ_SYNC_WORD_VALID)
		    {
		    }
		    else if (irq_mask & SX126X_IRQ_HEADER_VALID)
		    {
		    	_printf("[sx126x]: SX126X_IRQ_HEADER_VALID\n");
		    }
		    else if (irq_mask & SX126X_IRQ_HEADER_ERROR)
		    {
		    }
		    else if (irq_mask & SX126X_IRQ_CRC_ERROR)
		    {
		    	_printf("[sx126x]: SX126X_IRQ_CRC_ERROR\n");
		    }
		    else if (irq_mask & SX126X_IRQ_CAD_DONE)
		    {
		    	_printf("[sx126x]: SX126X_IRQ_CAD_DONE\n");
		    }
		    else if (irq_mask & SX126X_IRQ_CAD_DETECTED)
		    {
		    	_printf("[sx126x]: SX126X_IRQ_CAD_DETECTED\n");
		    }
		    else if (irq_mask & SX126X_IRQ_TIMEOUT)
		    {
		    	_printf("[sx126x]: SX126X_IRQ_TIMEOUT\n");
		    }
		    else
		    {
//		        _printf("[sx126x]: SX126X_IRQ_NONE\n");
		    }
		    NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
		}
	}
}
uint8_t sx126x_driver_receive(uint8_t* data, uint8_t max_len)
{
    sx126x_rx_buffer_status_t rx_buffer_status;
    sx126x_set_standby(sx126x_dev, SX126X_STANDBY_CFG_RC);
    sx126x_get_rx_buffer_status(sx126x_dev, &rx_buffer_status);
    uint8_t len = rx_buffer_status.pld_len_in_bytes;
    if(len > max_len)
    	len = max_len;
    sx126x_read_buffer(sx126x_dev,rx_buffer_status.buffer_start_pointer,data,len);
    return len;
}
