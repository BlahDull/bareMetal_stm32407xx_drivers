
# bareMetal_stm32407xx_drivers

Project implementing bare metal drivers for peripheral control for STM32407xx family. Written entirely in C with direct register manipulation and without the use of a HAL. Offers several APIs to allow user defined programs to utilize the drivers inside the application layer for their own programs.

**Drivers Implemented:**

* **GPIO**: Allows the user to specify several configurations for the pins on each GPIO port of the MCU. Such as speed, output type, pull up / pull down resistors, alternate functionality mode, etc.

* **SPI**: Allows the user to use SPI protocol in their applications for serial communication with sensors or other devices. The user can specify clock speed, CPOL and CPHA configuration, Master/Slave mode, and whether to use 8 or 16 bit communication. Supports full-duplex, half-duplex, or simplex communication.

* **I2C**: Allows the user to use I2C protocol in their applications for serial communication with sensors or other devices. The user can specify speed, addresses, whether or not to use automatic ACKing, the mode (standard or fast), and supports duty cycles of 2/1 and 16/9.

* **USART**: TODO

# API Description & Usage
***TODO***

# Installation

Standard compilation routine using ARM compiler for STM32F407xx family of devices is sufficient for compiling the drivers.

**Example**
> *arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -std=gnu11 -O0 stm32f407xx.gpio_driver.c -o stm32f407xx.gpio_driver.o*

# Implementation

C was the exclusive programming language used for this project. No libraries were used aside from C standard libraries like stdint.h and stddef.h. Utilizing the datasheet and reference manual for the STM32407xx family of MCUs, I wrote several macros and functions that use direct hardware level access to implement the different protocols using the peripherals in the MCU.

# Background
This project was taken for the purpose of learning and getting hands on experience with bare metal embedded driver development. I have enjoy low-level development and wanted to gain experience working with register level programming and implementing my own APIs for different protocols without using a HAL, to learn how the hardware/software interactions really work.

# Known Issues

***TODO***