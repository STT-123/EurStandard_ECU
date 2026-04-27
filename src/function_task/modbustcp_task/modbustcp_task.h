#ifndef __C_MODBUS_SERVER_H__
#define __C_MODBUS_SERVER_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "interface/modbus/modbus.h"
#include "interface/modbus/modbus-tcp.h"
#include "device_drv/abncheck/abncheck.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/if.h>

extern uint16_t *modbusBuff;

#define NB_CONNECTION 3
extern const uint16_t REGISTERS_START_ADDRESS;
extern char modbus_ip[16];
void modbusTcpServerTaskCreate(void);
int get_timeout_flag();

#endif
