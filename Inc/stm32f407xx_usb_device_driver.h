
#ifndef STM32F407XX_USB_DEVICE_DRIVER_H_
#define STM32F407XX_USB_DEVICE_DRIVER_H_

#include "stm32f407xx.h"

#define OTG_HS_GUSBCFG ((uint32_t*)(USB_OTG_HS_BASE_ADDR + 0x00CU))
#define OTG_HS_DCFG    ((uint32_t*)(USB_OTG_HS_BASE_ADDR + 0x800U))
#define OTG_HS_GCCFG   ((uint32_t*)(USB_OTG_HS_BASE_ADDR + 0x038U))
#define OTG_HS_GINTMSK ((uint32_t*)(USB_OTG_HS_BASE_ADDR + 0x018U))
#define OTG_HS_GINTSTS ((uint32_t*)(USB_OTG_HS_BASE_ADDR + 0x014U))
#define OTG_HS_GAHBCFG ((uint32_t*)(USB_OTG_HS_BASE_ADDR + 0x008U))
#define OTG_HS_DCTL    ((uint32_t*)(USB_OTG_HS_BASE_ADDR + 0x804U))

#define OTG_HS_DEVICE_SPEED_HS 0b00U
#define OTG_HS_DEVICE_SPEED_FS_EPHY 0b01U
#define OTG_HS_DEVICE_SPEED_FS_IPHY 0b11U


void USB_HS_Device_Init_GPIO(void);
void USB_HS_Device_Initialize_Core(void);
void USB_HS_Device_ConnectBus(void);
void USB_HS_Device_DisconnectBus(void);

#endif
