/*
 * sai.h
 *
 *  Created on: Apr 20, 2017
 *      Author: petera
 */

#ifndef SRC_SAI_H_
#define SRC_SAI_H_

#include "system.h"

#define PORT_SAI_A_FS     GPIOE
#define PIN_SAI_A_FS      GPIO_PIN_4
#define ALT_SAI_A_FS      GPIO_AF6_SAI1
#define PORT_SAI_A_SCK    GPIOE
#define PIN_SAI_A_SCK     GPIO_PIN_5
#define ALT_SAI_A_SCK     GPIO_AF6_SAI1
#define PORT_SAI_A_SD     GPIOE
#define PIN_SAI_A_SD      GPIO_PIN_6
#define ALT_SAI_A_SD      GPIO_AF6_SAI1
#define PORT_SAI_A_MCLK   GPIOE
#define PIN_SAI_A_MCLK    GPIO_PIN_2
#define ALT_SAI_A_MCLK    GPIO_AF6_SAI1

#define PORT_SAI_B_FS     GPIOF
#define PIN_SAI_B_FS      GPIO_PIN_9
#define ALT_SAI_B_FS      GPIO_AF6_SAI1
#define PORT_SAI_B_SCK    GPIOF
#define PIN_SAI_B_SCK     GPIO_PIN_8
#define ALT_SAI_B_SCK     GPIO_AF6_SAI1
#define PORT_SAI_B_SD     GPIOE
#define PIN_SAI_B_SD      GPIO_PIN_3
#define ALT_SAI_B_SD      GPIO_AF6_SAI1
#define PORT_SAI_B_MCLK   GPIOF
#define PIN_SAI_B_MCLK    GPIO_PIN_7
#define ALT_SAI_B_MCLK    GPIO_AF6_SAI1


#endif /* SRC_SAI_H_ */
