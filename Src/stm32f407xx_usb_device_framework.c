/*
 * stm32f407xx_usb_device_framework.c
 *
 *  Created on: Apr 25, 2026
 *      Author: Blah
 */

#include "stm32f407xx_usb_device_framework.h"

void USB_Device_Init(void) {
	USB_HS_Device_Init_GPIO();
	USB_HS_Device_Initialize_Core();
	USB_HS_Device_ConnectBus();
}
