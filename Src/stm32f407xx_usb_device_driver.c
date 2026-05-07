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

static void USBRST_Handler(void) {
	for (size_t i = 0; i < ENDPOINT_COUNT; i++) {

	}
}

static inline uint32_t* Get_Device_Endpoint_Ctrl_Reg(uint8_t InOrOut, uint8_t ep_num) {
	if (InOrOut)
		return (uint32_t*) (USB_OTG_HS_BASE_ADDR + (0x900U + (0x20U * ep_num)));
	else
		return (uint32_t*) (USB_OTG_HS_BASE_ADDR + (0xB00U + (0x20U * ep_num)));
}

static inline uint32_t* Get_Device_Endpoint_Interrupt_Reg(uint8_t InOrOut, uint8_t ep_num) {
	if (InOrOut)
		return (uint32_t*) (USB_OTG_HS_BASE_ADDR + (0x908U + (0x20U * ep_num)));
	else
		return (uint32_t*) (USB_OTG_HS_BASE_ADDR + (0xB08U + (0x20U * ep_num)));
}

static inline uint32_t* Get_Device_IN_EP_FIFO_Reg(uint8_t ep_num) {
	return (uint32_t*) (USB_OTG_HS_BASE_ADDR + (0x104 + (0x04 * (ep_num - 1))));
}

static void Update_FIFO_Start_Addrs() {
	// get end of rxfifo
	uint16_t start = (*OTG_HS_GRXFSIZ & 0xFFFF) * 4;
	// update txfifo0
	*OTG_HS_TX0FSIZ &= ~(0xFFFF << OTG_HS_TX0FSIZ_NPTXFSA_POS);
	*OTG_HS_TX0FSIZ |= (start << OTG_HS_TX0FSIZ_NPTXFSA_POS);
	// get the next address
	start += (((*OTG_HS_TX0FSIZ) >> OTG_HS_TX0FSIZ_NPTXFD_POS) & 0xFFFF) * 4;
	// update
	for (uint8_t i = 0; i < ENDPOINT_COUNT - 1; i++) {
		uint32_t* curr_tx_fifo = Get_Device_IN_EP_FIFO_Reg(i);
		*curr_tx_fifo &= ~(0xFFFF);
		*curr_tx_fifo |= (start << OTG_HS_DIEPTXFx_INEPTXSA_POS);
		start += (((*curr_tx_fifo) >> OTG_HS_DIEPTXFx_INEPTXFD_POS) & 0xFFFF) * 4;
	}
}

static void Config_RXFifo_Size(uint16_t size) {
	uint16_t calc_size = 10 + (2 * ((size / 4) + 1));
	*OTG_HS_GRXFSIZ &= ~(0xFFFF);
	*OTG_HS_GRXFSIZ |= (calc_size);
	Update_FIFO_Start_Addrs();
}

static void Config_TXFifo_Size(uint8_t ep_num, uint16_t size) {
	uint16_t calc_size = (size + 3) / 4;
	if (!ep_num) {
		*OTG_HS_TX0FSIZ &= ~(0xFFFF << OTG_HS_TX0FSIZ_NPTXFD_POS);
		*OTG_HS_TX0FSIZ |= (calc_size << OTG_HS_TX0FSIZ_NPTXFD_POS);
	} else {
		uint32_t* OTG_HS_DIEPTXF = Get_Device_IN_EP_FIFO_Reg(ep_num);
		*OTG_HS_DIEPTXF &= ~(0xFFFF << OTG_HS_DIEPTXFx_INEPTXFD_POS);
		*OTG_HS_DIEPTXF |= (calc_size << OTG_HS_DIEPTXFx_INEPTXFD_POS);
	}
	Update_FIFO_Start_Addrs();
}

static void Config_IN_EP(uint8_t ep_num, uint8_t packet_size, uint8_t ep_type) {
	*OTG_HS_DAINTMSK |= (1 << ep_num);

	uint32_t* DIEPCTL_x = Get_Device_Endpoint_Ctrl_Reg(IN_EP, ep_num);
	*DIEPCTL_x &= ~(0b11111111111U << OTG_HS_DIEPCTL_MPSIZ_POS);
	*DIEPCTL_x &= ~(0b11U << OTG_HS_DIEPCTL_EPTYP_POS);
	*DIEPCTL_x |= (1 << OTG_HS_DIEPCTL_USBAEP_POS);
	*DIEPCTL_x |= (1 << OTG_HS_DIEPCTL_SNAK_POS);
	*DIEPCTL_x |= (1 << OTG_HS_DIEPCTL_USBAEP_POS);
	*DIEPCTL_x |= (1 << OTG_HS_DIEPCTL_SD0PID_SEVNFRM_POS);
	*DIEPCTL_x |= (packet_size << OTG_HS_DIEPCTL_MPSIZ_POS);
	*DIEPCTL_x |= (ep_type << OTG_HS_DIEPCTL_EPTYP_POS);
}

static void Deconfig_EP(uint8_t ep_num) {
	*OTG_HS_DAINTMSK &= ~(1 << ep_num);
	*OTG_HS_DAINTMSK &= ~((1 << 16) << ep_num);

	uint32_t* OTG_HS_DIEPINT = Get_Device_Endpoint_Interrupt_Reg(IN_EP, ep_num);
	uint32_t* OTG_HS_DOEPINT = Get_Device_Endpoint_Interrupt_Reg(OUT_EP, ep_num);

	uint32_t* OTG_HS_DIEPCTL = Get_Device_Endpoint_Ctrl_Reg(IN_EP, ep_num);
	uint32_t* OTG_HS_DOEPCTL = Get_Device_Endpoint_Ctrl_Reg(OUT_EP, ep_num);

	*OTG_HS_DIEPINT |= (0x29FF);
	*OTG_HS_DOEPINT |= (0x71FF);

	if (*OTG_HS_DIEPCTL & (1 << OTG_HS_DIEPCTL_EPENA_POS)) {
		*OTG_HS_DIEPCTL &= ~(1 << OTG_HS_DIEPCTL_EPDIS_POS);
	}

	*OTG_HS_DIEPCTL &= ~(1 << OTG_HS_DIEPCTL_USBAEP_POS);

	if (ep_num != 0) {
		if (*OTG_HS_DOEPCTL & (1 << OTG_HS_DOEPCTL_EPENA_POS)) {
			*OTG_HS_DOEPCTL &= ~(1 << OTG_HS_DOEPCTL_EPDIS_POS);
		}
		*OTG_HS_DOEPCTL &= ~(1 << OTG_HS_DOEPCTL_USBAEP_POS);
	}

}

static void Config_EP0(uint8_t packet_size) {
	// un-mask interrupts for ep0
	*OTG_HS_DAINTMSK |= (1 << 0);
	*OTG_HS_DAINTMSK |= (1 << (16 % 16));

	uint32_t* DIEPCTL_0 = Get_Device_Endpoint_Ctrl_Reg(IN_EP,0);
	uint32_t* DOEPCTL_0 = Get_Device_Endpoint_Ctrl_Reg(OUT_EP, 0);

	*DIEPCTL_0 &= ~(0b11111111111U << OTG_HS_DIEPCTL_MPSIZ_POS);
	*DIEPCTL_0 |= (packet_size << OTG_HS_DIEPCTL_MPSIZ_POS);
	*DIEPCTL_0 |= (1 << OTG_HS_DIEPCTL_USBAEP_POS);
	*DIEPCTL_0 |= (1 << OTG_HS_DIEPCTL_SNAK_POS);

	*DOEPCTL_0 |= (1 << OTG_HS_DOEPCTL_CNAK_POS);
	*DOEPCTL_0 |= (1 << OTG_HS_DOEPCTL_EPENA_POS);



}

void USB_HS_Device_GINTSTS_IRQHandling(void) {
	__vo uint32_t global_interrupts = *OTG_HS_GINTSTS;
	if (global_interrupts & (1 << OTG_HS_GINTMSK_ENUMDNEM_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_ENUMDNEM_POS);
	}
	if (global_interrupts & (1 << OTG_HS_GINTMSK_IEPINT_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_IEPINT_POS);
	}
	if (global_interrupts & (1 << OTG_HS_GINTMSK_RXFLVLM_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_RXFLVLM_POS);
	}
	if (global_interrupts & (1 << OTG_HS_GINTMSK_SOFM_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_SOFM_POS);
	}
	if (global_interrupts & (1 << OTG_HS_GINTMSK_USBRST_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_USBRST_POS);

	}
	if (global_interrupts & (1 << OTG_HS_GINTMSK_USBSUSPM_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_USBSUSPM_POS);
	}
	if (global_interrupts & (1 << OTG_HS_GINTMSK_WUIM_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_WUIM_POS);
	}
	if (global_interrupts & (1 << OTG_HS_GINTMSK_OEPINT_POS)) {
		*OTG_HS_GINTSTS |= (1 << OTG_HS_GINTMSK_OEPINT_POS);
	}
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
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_OEPINT_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_RXFLVLM_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_SOFM_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_USBRST_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_USBSUSPM_POS);
	*OTG_HS_GINTMSK |= (1 << OTG_HS_GINTMSK_WUIM_POS);

	// Un-mask the global interrupt
	*OTG_HS_GAHBCFG |= (1 << OTG_HS_GAHBCFG_GINT_POS);
}
