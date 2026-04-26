/*
 * stm32f407xx_usb_device_driver.c
 *
 *  Created on: Apr 25, 2026
 *      Author: Blah
 */

#include "stm32f407xx_usb_device_driver.h"

void USB_HS_Device_Init_GPIO(void) {
	GPIO_Handle_t handle = {0};
	handle.pGPIOx = GPIOB;
	handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	handle.GPIO_PinConfig.GPIO_PinAltFunMode = 12;
	handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14; // DM/D-
	GPIO_Init(&handle);
	handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15; // DP/D+
	GPIO_Init(&handle);
}

void USB_HS_Device_ConnectBus(void) {
	*OTG_HS_GCCFG |= (1 << OTG_HS_GCCFG_PWRDWN_POS);
	*OTG_HS_DCTL &= ~(1 << OTG_HS_DCTL_SDIS_POS);
}

void USB_HS_Device_DisconnectBus(void) {
	*OTG_HS_DCTL |= (1 << OTG_HS_DCTL_SDIS_POS);
	*OTG_HS_GCCFG &= ~(1 << OTG_HS_GCCFG_PWRDWN_POS);
}

void USB_HS_Device_Initialize_Core(void) {
	USB_HS_PCLK_EN();
	// Set forced peripheral mode
	*OTG_HS_GUSBCFG |= (1 << OTG_HS_GUSBCFG_FDMOD_POS);
	// Set physical layer
	*OTG_HS_GUSBCFG |= (1 << OTG_HS_GUSBCFG_PHYSEL_POS);
	// Set turn around time
	*OTG_HS_GUSBCFG &= ~(0b1111U << OTG_HS_GUSBCFG_TRDT_POS);
	*OTG_HS_GUSBCFG |= (0x9U << OTG_HS_GUSBCFG_TRDT_POS);
	// Sets device speed as Full speed with internal PHY
	*OTG_HS_DCFG |= (OTG_HS_DEVICE_SPEED_FS_IPHY << OTG_HS_DCFG_DSPD_POS);
	// Set the VBUS sensing on
	*OTG_HS_GCCFG |= (1 << OTG_HS_GCCFG_VBUSBSEN_POS);
	// Clear interrupt register
	*OTG_HS_GINTSTS |= (0xFFFFFFFF);
	// Un-mask interrupts
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_ENUMDNEM_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_IEPINT_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_RXFLVLM_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_SOFM_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_USBRST_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_USBSUSPM_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_WUIM_POS);

	// Un-mask the global interrupt
	*OTG_HS_GAHBCFG |= (1 << OTG_HS_GAHBCFG_GINT_POS);
}
