/*
 * pppos_netif.c
 *
 *  Created on: 7 янв. 2024 г.
 *      Author: klime
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "netif/ppp/pppos.h"
#include "netif/ppp/ppp.h"
#include "main.h"

static void status_cb(ppp_pcb *pcb, int err_code, void *ctx);
static u32_t output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx);
/* The PPP control block */
ppp_pcb *ppp;

/* The PPP IP interface */
struct netif ppp_netif;

uint8_t rxbuffer[250];
void pppos_rx_thread(void* ctx)
{
	uint16_t len;
	usart_t* dev = USART_GetDev(USART2);
	while(1)
	{
		len = USART_Receive(dev,rxbuffer,250);
		if(len > 0)
		{
			pppos_input_tcpip(ppp, rxbuffer, len);
		}
		vTaskDelay(5);
	}
}
static void ctx_cb(void){

}
void pppos_thread(void* ctx)
{
	ppp = pppos_create(&ppp_netif,
	       output_cb, status_cb, ctx_cb);

	ppp_set_default(ppp);

//	ip4_addr_t addr;
//
//	/* Set our address */
//	IP4_ADDR(&addr, 192,168,0,13);
//	ppp_set_ipcp_ouraddr(ppp, &addr);
//
//	/* Set peer(his) address */
//	IP4_ADDR(&addr, 192,168,0,21);
//	ppp_set_ipcp_hisaddr(ppp, &addr);

	//ppp_set_silent(ppp, 1);

	/*
	 * Initiate PPP listener (i.e. wait for an incoming connection), can only
	 * be called if PPP session is in the dead state (i.e. disconnected).
	 */
	//ppp_listen(ppp);
	ppp_connect(ppp,0);
	while(1)
	{

		vTaskDelay(1000);
	}
}
static u32_t output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
	usart_t* dev = USART_GetDev(USART2);
  return USART_Transmit(dev, data, len);
}

void pppos_init(void)
{
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
	USART_Transmit(dev,"bebra\r\n",sizeof("bebra\r\n"));
	config.base = USART2;
	config.gpio = GPIOA;
	config.rx_pin = 2;
	config.tx_pin = 3;
	USART_Init(&config);

	xTaskCreate(pppos_rx_thread,"pppos_rx_thread",configMINIMAL_STACK_SIZE*2,(void*)NULL,configMAX_PRIORITIES - 3,(void*)NULL);
	xTaskCreate(pppos_thread,"pppos_thread",configMINIMAL_STACK_SIZE,(void*)NULL,configMAX_PRIORITIES - 3,(void*)NULL);

}

static void status_cb(ppp_pcb *pcb, int err_code, void *ctx) {
  struct netif *pppif = ppp_netif(pcb);
  LWIP_UNUSED_ARG(ctx);

  switch(err_code) {
    case PPPERR_NONE: {
#if LWIP_DNS
      const ip_addr_t *ns;
#endif /* LWIP_DNS */
      printf2("status_cb: Connected\n");
#if PPP_IPV4_SUPPORT
      printf2("   our_ipaddr  = %s\n", ipaddr_ntoa(&pppif->ip_addr));
      printf2("   his_ipaddr  = %s\n", ipaddr_ntoa(&pppif->gw));
      printf2("   netmask     = %s\n", ipaddr_ntoa(&pppif->netmask));
#if LWIP_DNS
      ns = dns_getserver(0);
      printf2("   dns1        = %s\n", ipaddr_ntoa(ns));
      ns = dns_getserver(1);
      printf2("   dns2        = %s\n", ipaddr_ntoa(ns));
#endif /* LWIP_DNS */
#endif /* PPP_IPV4_SUPPORT */
#if PPP_IPV6_SUPPORT
      printf2("   our6_ipaddr = %s\n", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));

#endif /* PPP_IPV6_SUPPORT */
      break;
    }
    case PPPERR_PARAM: {
      printf2("status_cb: Invalid parameter\n");
      break;
    }
    case PPPERR_OPEN: {
      printf2("status_cb: Unable to open PPP session\n");
      break;
    }
    case PPPERR_DEVICE: {
      printf2("status_cb: Invalid I/O device for PPP\n");
      break;
    }
    case PPPERR_ALLOC: {
      printf2("status_cb: Unable to allocate resources\n");
      break;
    }
    case PPPERR_USER: {
      printf2("status_cb: User interrupt\n");
      break;
    }
    case PPPERR_CONNECT: {
      printf2("status_cb: Connection lost\n");

      break;
    }
    case PPPERR_AUTHFAIL: {
      printf2("status_cb: Failed authentication challenge\n");
      break;
    }
    case PPPERR_PROTOCOL: {
      printf2("status_cb: Failed to meet protocol\n");
      break;
    }
    case PPPERR_PEERDEAD: {
      printf2("status_cb: Connection timeout\n");
      break;
    }
    case PPPERR_IDLETIMEOUT: {
      printf2("status_cb: Idle Timeout\n");
      break;
    }
    case PPPERR_CONNECTTIME: {
      printf2("status_cb: Max connect time reached\n");
      break;
    }
    case PPPERR_LOOPBACK: {
      printf2("status_cb: Loopback detected\n");
      break;
    }
    default: {
      printf2("status_cb: Unknown error code %d\n", err_code);
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
