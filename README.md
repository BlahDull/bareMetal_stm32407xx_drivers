
# bareMetal_stm32407xx_drivers

Project implementing bare metal drivers for peripheral control for STM32407xx family. Written entirely in C with direct register manipulation and without the use of a HAL. Offers several APIs to allow user defined programs to utilize the drivers inside the application layer for their own programs.

**Drivers Implemented:**

* **GPIO**: Allows the user to specify several configurations for the pins on each GPIO port of the MCU. Such as speed, output type, pull up / pull down resistors, alternate functionality mode, etc. Supports polling and interrupt modes.

* **SPI**: Allows the user to use SPI protocol in their applications for serial communication with sensors or other devices. The user can specify clock speed, CPOL and CPHA configuration, Master/Slave mode, and whether to use 8 or 16 bit communication. Supports full-duplex, half-duplex, or simplex communication. Supports polling and interrupt modes.

* **I2C**: Allows the user to use I2C protocol in their applications for serial communication with sensors or other devices. The user can specify speed, addresses, whether or not to use automatic ACKing, the mode (standard or fast), and supports duty cycles of 2/1 and 16/9. Supports polling and interrupt modes.

* **USART**: Allows the user to use UART protocol in their applications for serial communication with sensors or other devices. The user can specify baud rate, number of stop bits, word length, etc. Supports polling, interrupt, and DMA modes.

* **DMA**: Allows the user to configure the DMA controller to enable DMA streams to transfer data from memory to memory, peripheral to memory, and memory to peripheral. The user can specify whether or not to use FIFO, the FIFO threshold, priority, whether or not to auto increment the addresses, and whether or not interrupts.

# API Description & Usage
Each driver gives a handle structure that the user should configure before calling the appropriate Init() function and passing the handle. In the case of SPI, I2C, and UART, the peripheral should then be enabled by calling the appropriate enable function.

# Installation

Standard compilation routine using ARM compiler for STM32F407xx family of devices is sufficient for compiling the drivers.

**Example**
> *arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -std=gnu11 -O0 stm32f407xx.gpio_driver.c -o stm32f407xx.gpio_driver.o*

# Implementation

C was the exclusive programming language used for this project. No libraries were used aside from C standard libraries like stdint.h and stddef.h. Utilizing the datasheet and reference manual for the STM32407xx family of MCUs, I wrote several macros and functions that use direct hardware level access to implement the different protocols using the peripherals in the MCU.

# Background
This project was taken for the purpose of learning and getting hands on experience with bare metal embedded driver development. I wanted to gain more experience working with register level programming and implementing my own APIs for different protocols without using a HAL, to learn how the hardware/software interactions really work. Going through the reference manual, datasheet, and schematics for the MCU was very interesting and provided a lot of insight on how
the software and hardware interactions take place.

# Known Issues
I2C interrupt can get stuck sending data if more than one start and stop condition is sent in quick sucession.

# ***TODO***
Implement DMA support for SPI and I2C.
