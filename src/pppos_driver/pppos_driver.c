/*
 * pppos_driver.c
 *
 *  Created on: 25 апр. 2024 г.
 *      Author: hocok
 */

#include "lwip/tcpip.h"
#include "netif/ppp/pppos.h"
#include "netif/ppp/ppp.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

ppp_pcb *ppp;
struct netif ppp_netif;

static u32_t output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx);
static void status_cb(ppp_pcb *pcb, int err_code, void *ctx);
static void ctx_cb(void);

void pppos_rx_thread(void* ctx)
{
	uint16_t len;
	usart_t* dev = USART_GetDev(USART2);
	uint8_t rxbuffer[128];
	while(1)
	{
		len = USART_Receive(dev,rxbuffer,128);
		if(len > 0)
		{
			pppos_input_tcpip(ppp, rxbuffer, len);
		}
		vTaskDelay(5);
	}
}
void pppos_driver_init(void* ctx)
{
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


	ppp = pppos_create(&ppp_netif,
	       output_cb, status_cb, ctx_cb);

	ppp_set_default(ppp);
	xTaskCreate(pppos_rx_thread,"pppos_rx",configMINIMAL_STACK_SIZE*2,(void*)NULL,2,(void*)NULL);
	ppp_connect(ppp,0);
}
static void ctx_cb(void)
{

}
static u32_t output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
	usart_t* dev = USART_GetDev(USART2);
  return USART_Transmit(dev, data, len);
}
static void status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
  struct netif *pppif = ppp_netif(pcb);
  LWIP_UNUSED_ARG(ctx);

  switch(err_code) {
    case PPPERR_NONE: {
#if LWIP_DNS
      const ip_addr_t *ns;
#endif /* LWIP_DNS */
      _printf("status_cb: Connected\n");
#if PPP_IPV4_SUPPORT
      _printf("   our_ipaddr  = %s\n", ipaddr_ntoa(&pppif->ip_addr));
      _printf("   his_ipaddr  = %s\n", ipaddr_ntoa(&pppif->gw));
      _printf("   netmask     = %s\n", ipaddr_ntoa(&pppif->netmask));
#if LWIP_DNS
      ns = dns_getserver(0);
      _printf("   dns1        = %s\n", ipaddr_ntoa(ns));
      ns = dns_getserver(1);
      _printf("   dns2        = %s\n", ipaddr_ntoa(ns));
#endif /* LWIP_DNS */
#endif /* PPP_IPV4_SUPPORT */
#if PPP_IPV6_SUPPORT
      _printf("   our6_ipaddr = %s\n", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));

#endif /* PPP_IPV6_SUPPORT */
      break;
    }
    case PPPERR_PARAM: {
      _printf("status_cb: Invalid parameter\n");
      break;
    }
    case PPPERR_OPEN: {
      _printf("status_cb: Unable to open PPP session\n");
      break;
    }
    case PPPERR_DEVICE: {
      _printf("status_cb: Invalid I/O device for PPP\n");
      break;
    }
    case PPPERR_ALLOC: {
      _printf("status_cb: Unable to allocate resources\n");
      break;
    }
    case PPPERR_USER: {
      _printf("status_cb: User interrupt\n");
      break;
    }
    case PPPERR_CONNECT: {
      _printf("status_cb: Connection lost\n");

      break;
    }
    case PPPERR_AUTHFAIL: {
      _printf("status_cb: Failed authentication challenge\n");
      break;
    }
    case PPPERR_PROTOCOL: {
      _printf("status_cb: Failed to meet protocol\n");
      break;
    }
    case PPPERR_PEERDEAD: {
      _printf("status_cb: Connection timeout\n");
      break;
    }
    case PPPERR_IDLETIMEOUT: {
      _printf("status_cb: Idle Timeout\n");
      break;
    }
    case PPPERR_CONNECTTIME: {
      _printf("status_cb: Max connect time reached\n");
      break;
    }
    case PPPERR_LOOPBACK: {
      _printf("status_cb: Loopback detected\n");
      break;
    }
    default: {
      _printf("status_cb: Unknown error code %d\n", err_code);
      break;
    }
  }

/*
 * This should be in the switch case, this is put outside of the switch
 * case for example readability.
 */

  if (err_code == PPPERR_NONE) {
    return;
  }

  /* ppp_close() was previously called, don't reconnect */
  if (err_code == PPPERR_USER) {
     ppp_free(pcb);
    return;
  }

  /*
   * Try to reconnect in 3 seconds, if you need a modem chatscript you have
   * to do a much better signaling here ;-)
   */
  ppp_connect(pcb, 3);
  /* OR ppp_listen(pcb); */
}
