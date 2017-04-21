/*
 * dac.h
 *
 *  Created on: Apr 21, 2017
 *      Author: petera
 */

#ifndef SRC_DAC_H_
#define SRC_DAC_H_

#include "system.h"

// DACOUT1 DMA1 Channel7 Stream5
#define PORT_DAC          GPIOA
#define PIN_DAC           GPIO_PIN_4

void dac_init(void);
void dac_start(u8_t *data, u32_t len);
void dac_stop(void);
void dac_sync(void);
void dac_deinit(void);

#endif /* SRC_DAC_H_ */
