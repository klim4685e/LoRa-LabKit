#include "stm32wle5xx.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "task.h"

#include "pppos_netif.h"
#include "usart.h"
#include "lwip/tcpip.h"

#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/api.h"

#include "main.h"

#include "sx126x_hal.h"

void vMainTask(void* ctx);




int main(void)
{
	SystemCoreClockUpdate();
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE4);
	GPIOB->MODER |= GPIO_MODER_MODE5_0 | GPIO_MODER_MODE4_0;
	GPIOB->BSRR = GPIO_BSRR_BR5;




	while(1)
	{
		xTaskCreate(vMainTask,"biba_boba",configMINIMAL_STACK_SIZE,(void*)NULL,TCPIP_THREAD_PRIO-2,(void*)NULL);
		vTaskStartScheduler();
		while(1)
		{};
	}
	return 0;

}
void vMainTask(void* ctx)
{
	tcpip_init(pppos_init,NULL);
	vTaskDelay(500);
	sys_thread_new("sockex_testrecv", vMainTask1, NULL, configMINIMAL_STACK_SIZE*4, TCPIP_THREAD_PRIO);
	while(1)
	{
		GPIOB->BSRR = GPIO_BSRR_BR4;
		vTaskDelay(500);

		GPIOB->BSRR = GPIO_BSRR_BS4;
		vTaskDelay(500);
	}
}

void vMainTask1(void* ctx)
{
	struct netconn *conn;
	ip6_addr_t* addr;
	struct netbuf *buf;
	unsigned short port;
	err_t err, recv_err;
	  conn = netconn_new(NETCONN_UDP_IPV6);
	  err = netconn_bind(conn, IP6_ADDR_ANY, 7);
	  if(err == ERR_OK)
	  {

	  }
	  else
	  {
		  netconn_delete(conn);
	  }
	  while(1)
	  {

		  recv_err = netconn_recv(conn,&buf);
		  if(recv_err == ERR_OK)
		  {
			  addr = netbuf_fromaddr(buf);
			  port = netbuf_fromport(buf);
			  netconn_connect(conn, addr, port);
			  netbuf_ref(buf, "test_respond\r\n", sizeof("test_respond\r\n"));
			  netconn_send(conn,buf);
			  netconn_disconnect(conn);
			  netbuf_delete(buf);
		  }
	  vTaskDelay(1000);
	  }
}
