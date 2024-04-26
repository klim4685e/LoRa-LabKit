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
#include "sx126x.h"

typedef struct sx126x sx126x_t;

typedef enum {
    STATE_IDLE,
    STATE_TX,
    STATE_ACK,
    STATE_RX,
    STATE_CCA_CLEAR,
    STATE_CCA_BUSY,
} sx126x_state_t;

typedef enum {
    SX126X_RF_MODE_RX,
    SX126X_RF_MODE_TX_LPA,
    SX126X_RF_MODE_TX_HPA,
} sx126x_rf_mode_t;

typedef struct {
    SPI_TypeDef* spi;
    sx126x_reg_mod_t regulator;

    void(*set_rf_mode)(sx126x_t *dev, sx126x_rf_mode_t rf_mode);

} sx126x_params_t;

typedef struct sx126x
{
    sx126x_params_t *params;
    sx126x_pkt_params_lora_t pkt_params;
    sx126x_mod_params_lora_t mod_params;
    uint32_t channel;
    uint16_t rx_timeout;
    bool radio_sleep;
    sx126x_cad_params_t cad_params;
    bool cad_detected;

    bool ifs        : 1;
    bool cca_send   : 1;
    bool ack_filter : 1;
    bool promisc    : 1;
    bool pending    : 1;

    uint8_t size;                           /**< size of the last received packet */
    sx126x_state_t state;

    uint8_t seq_num;

//    uint8_t short_addr[IEEE802154_SHORT_ADDRESS_LEN];    /**< Short (2 bytes) device address */
//    uint8_t long_addr[IEEE802154_LONG_ADDRESS_LEN];     /**< Long (8 bytes) device address */
    uint16_t pan_id;
}sx126x_t;



#endif /* SRC_SX126X_DRIVER_SX126X_DRIVER_H_ */
