Pet-project of the IoT node on the LoRa Labkit RAK3172 (STM32WLE5CC). SibSAU. 
Based on LwIP 6LoWPAN + LoRa PHY.
Base station should be connected through PPPoS (?) to a PC.

Here is a to-do list:
- [x] FreeRTOS based minimal project with make script and CMSIS. NO STM32CUBEMX HAL.
- [ ] USART Communication for PPPoS. Transmitting should be impremented with DMA. Receiving - ring buffer (?)
- [ ] Create pppos netif. Connect a PC with the node (pppd in linux, windows idk :( ) PING SHOULD BE PASSED
- [ ] LoRa driver. 
- [ ] Create 6LoWPAN Adaptation layer. First step is the passing ping.
- [ ] Create the bridge between netifs.

 
