/*
 * sx126x_driver.h
 *
 *  Created on: 11 апр. 2024 г.
 *      Author: hocok
 */

#ifndef SRC_SX126X_DRIVER_SX126X_DRIVER_H_
#define SRC_SX126X_DRIVER_SX126X_DRIVER_H_


#include "stm32wlxx.h"
#include <stdint.h>

typedef struct sx126x
{
	uint8_t radio_sleep;
}sx126x_t;



#endif /* SRC_SX126X_DRIVER_SX126X_DRIVER_H_ */
