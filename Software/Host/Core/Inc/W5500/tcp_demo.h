#ifndef __TCP_DEMO_H
#define __TCP_DEMO_H
#include "types.h"
#include "stm32f1xx_hal.h"

extern uint16 W5500_tcp_server_port;
void do_tcp_server(void);//TCP Server回环演示函数
void do_tcp_client(uint8_t VoltageD[], uint16_t len);//TCP Clinet回环演示函数
#endif 

