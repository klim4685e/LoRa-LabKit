/*
 * sx126x_hal.c
 *
 *  Created on: 22 февр. 2024 г.
 *      Author: klime
 */

typedef struct
{

}sx126x_t;

#include "stm32wlxx.h"
#include "sx126x_hal.h"

uint8_t sx126x_setup(sx126x_t* dev)
{
	RCC->APB3ENR |= RCC_APB3ENR_SUBGHZSPIEN;
	RCC->APB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->APB2ENR |= RCC_AHB2ENR_GPIOCEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODE4);
	GPIOA->MODER |=  GPIO_MODER_MODE4_0;

	GPIOB->MODER &= ~(GPIO_MODER_MODE8);
	GPIOB->MODER |=  GPIO_MODER_MODE8_0;

	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
	GPIOC->MODER |=  GPIO_MODER_MODE13_0;

	DMAMUX1_Channel5->CCR = 41; //SUBGHZSPI_RX
	DMA1_Channel5->CPAR = SUBGHZSPI->DR;
	DMAMUX1_Channel6->CCR = 42; //SUBGHZSPI_TX
	DMA1_Channel6->CPAR = SUBGHZSPI->DR;

	SUBGHZSPI->CR1 |= SPI_CR1_SSM | SPI_CR1_SPE | SPI_CR1_MSTR;

	SUBGHZSPI->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
}
uint8_t sx126x_init(sx126x_t* dev)
{
	sx126x_setup(dev);
}
int8_t sx126x_send(sx126x_t* dev, uint8_t* data, uint8_t len)
{
	return len;
}
int8_t sx126x_recv(sx126x_t* dev, uint8_t* data)
{
	return 0;
}
