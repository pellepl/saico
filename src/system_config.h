#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

#include "types.h"
#include "config_header.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"


// enable uart
#define CONFIG_UART2
#define CONFIG_UART2_PORT       0
#define CONFIG_UART_CNT         1

/** TICKER **/
// STM32 system timer
#define CONFIG_STM32_SYSTEM_TIMER   		2
// system timer frequency
#define SYS_MAIN_TIMER_FREQ   			10000
// system timer counter type
typedef u16_t system_counter_type;
// system tick frequency
#define SYS_TIMER_TICK_FREQ   			1000
// os ticker cpu clock div
#define SYS_OS_TICK_DIV       			8

/** IO **/
#define CONFIG_DEFAULT_DEBUG_MASK   		(0xffffffff)
#define CONFIG_IO_MAX   			1

#define IOSTD        				0
#define IODBG        				IOSTD

#define LED_PORT            GPIOB
#define LED_PIN             GPIO_PIN_7

#define VALID_DATA(x)      1
#define VALID_RAM(x)       1

#include "miniutils.h"

#endif // _SYSTEM_CONFIG_H
