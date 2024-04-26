/*
 * sx126x_hal.c
 *
 *  Created on: 11 апр. 2024 г.
 *      Author: hocok
 */


#include "sx126x_hal.h"
#include "stm32wlxx.h"
#include "sx126x.h"
#include "sx126x_driver.h"
#include "usart.h"

sx126x_hal_status_t sx126x_hal_init(const void* context);
void sx126x_rf_switch_init(sx126x_t* dev);
void sx126x_rf_switch_set(sx126x_t* dev, sx126x_rf_mode_t rf_mode);

sx126x_t SX126X_DEV;
sx126x_t* sx126x_dev = &SX126X_DEV;

sx126x_params_t spi_params =
{
	.spi = SUBGHZSPI,
	.regulator = SX126X_REG_MODE_DCDC,
	.set_rf_mode = sx126x_rf_switch_set,
};
void subghzspi_debug_setup(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);

	GPIOA->AFR[0] |= ((13 << GPIO_AFRL_AFSEL4_Pos) | (13 << GPIO_AFRL_AFSEL5_Pos) | (13 << GPIO_AFRL_AFSEL6_Pos) | (13 << GPIO_AFRL_AFSEL7_Pos));
	sx126x_hal_init(sx126x_dev);

}
void subghzspi_init(void)
{
	RCC->APB3ENR |= RCC_APB3ENR_SUBGHZSPIEN;
	SUBGHZSPI->CR1 &= ~(SPI_CR1_SPE);
	SUBGHZSPI->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSI | (6 << SPI_CR1_BR_Pos) | SPI_CR1_SSM);
	SUBGHZSPI->CR2 |= (SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2);
	SUBGHZSPI->CR1 |= (SPI_CR1_SPE);
}
sx126x_hal_status_t sx126x_hal_init(const void* context)
{
	sx126x_t* dev = (sx126x_t*)context;
	dev->params = &spi_params;
	subghzspi_init();
	sx126x_hal_wakeup(dev);
	sx126x_hal_reset(dev);
	sx126x_rf_switch_init(sx126x_dev);
}
static uint8_t sx126x_radio_wait_until_ready(sx126x_t *dev)
{
    if (dev->radio_sleep == true) {
        _printf("Wakeup radio\n");
        sx126x_hal_wakeup(dev);
    }

    while (((PWR->SR2 & PWR_SR2_RFBUSYMS) && ((PWR->SR2 & PWR_SR2_RFBUSYS))) == 1) {}
    return 0;
}

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
	sx126x_t *dev = (sx126x_t *)context;
	sx126x_radio_wait_until_ready(dev);

	if(command[0] == 0x84 || command[0] == 0x94)
	{
		dev->radio_sleep = true;
	}
	else
	{
		dev->radio_sleep = false;
	}
	PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS;

	for(uint8_t i = 0; i < command_length; i++)
	{
		while((SUBGHZSPI->SR & SPI_SR_TXE) != SPI_SR_TXE) { __asm("nop");}
		__IO uint8_t *spidr = ((__IO uint8_t*)&SUBGHZSPI->DR);
		*spidr = command[i];
		while((SUBGHZSPI->SR & SPI_SR_RXNE) != SPI_SR_RXNE) { __asm("nop");}
		(void)SUBGHZSPI->DR;
	}
	if(data_length > 0)
	{
		for(uint8_t i = 0; i < data_length; i++)
		{
			while((SUBGHZSPI->SR & SPI_SR_TXE) != SPI_SR_TXE) { __asm("nop");}
			__IO uint8_t *spidr = ((__IO uint8_t*)&SUBGHZSPI->DR);
			*spidr = data[i];

			while((SUBGHZSPI->SR & SPI_SR_RXNE) != SPI_SR_RXNE) { __asm("nop");}
			(void)SUBGHZSPI->DR;
		}
	}
	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;
	return SX126X_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
	sx126x_t *dev = (sx126x_t *)context;
	sx126x_radio_wait_until_ready(dev);

	PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS;

	for(uint8_t i = 0; i < command_length; i++)
	{
		while((SUBGHZSPI->SR & SPI_SR_TXE) != SPI_SR_TXE) { __asm("nop");}
		__IO uint8_t *spidr = ((__IO uint8_t*)&SUBGHZSPI->DR);
		*spidr = command[i];
		while((SUBGHZSPI->SR & SPI_SR_RXNE) != SPI_SR_RXNE) { __asm("nop");}
		(void)SUBGHZSPI->DR;
	}
	for(uint8_t i = 0; i < data_length; i++)
	{
		while((SUBGHZSPI->SR & SPI_SR_TXE) != SPI_SR_TXE) { __asm("nop");}
		__IO uint8_t *spidr = ((__IO uint8_t*)&SUBGHZSPI->DR);
		*spidr = 0x55;

		while((SUBGHZSPI->SR & SPI_SR_RXNE) != SPI_SR_RXNE) { __asm("nop");}
		data[i] = (uint8_t)SUBGHZSPI->DR;
	}
	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;
	return SX126X_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
	_printf("sx126x_hal_reset\n");
	sx126x_t *dev = (sx126x_t *)context;


	RCC->CSR |= RCC_CSR_RFRST;
	RCC->CSR &= ~RCC_CSR_RFRST;

	vTaskDelay(1);
	while ((RCC->CSR & RCC_CSR_RFRSTF) != 0UL) {}

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;

	EXTI->IMR2 |= EXTI_IMR2_IM44;

	NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0);
	NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

	PWR->CR3 |= PWR_CR3_EWRFBUSY;
	PWR->SCR = PWR_SCR_CWRFBUSYF;

	dev->radio_sleep = true;
	return SX126X_STATUS_OK;
}
sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
	_printf("sx126x_hal_wakeup\n");
    PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS;
    vTaskDelay(1);
    /* Pull NSS high */
    PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;
    vTaskDelay(1);
    return SX126X_STATUS_OK;
}

void sx126x_rf_switch_init(sx126x_t* dev)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	GPIOB->MODER &= ~GPIO_MODER_MODE8_1;
	GPIOC->MODER &= ~GPIO_MODER_MODE13_1;
}
void sx126x_rf_switch_set(sx126x_t* dev, sx126x_rf_mode_t rf_mode)
{
	switch (rf_mode)
	{
	case SX126X_RF_MODE_RX:
	{
		GPIOB->BSRR = GPIO_BSRR_BS8;
		GPIOC->BSRR = GPIO_BSRR_BR13;
		break;
	}
	case SX126X_RF_MODE_TX_HPA:
	case SX126X_RF_MODE_TX_LPA:
	{
		GPIOB->BSRR = GPIO_BSRR_BR8;
		GPIOC->BSRR = GPIO_BSRR_BS13;
		break;
	}
	default:
		break;
	}
}

