/*
 * app.c
 *
 *  Created on: Apr 8, 2017
 *      Author: petera
 */

#ifndef SRC_APP_C_
#define SRC_APP_C_

#include "app.h"
#include "taskq.h"
#include "dac.h"


static task_timer led_timer1;
static task_timer led_timer2;
static void led_blinky_on(uint32_t state, void *i_p);
static task *led_task;

static void led_blinky_on(uint32_t state, void *i_p) {
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, state);
  //print("blink\n");
  if (state) {
    TASK_start_timer(led_task, &led_timer2, 0, NULL, 10, 0, "led0");
  }
}

void app_init(void) {
  // create led blink timer
  led_task = TASK_create(led_blinky_on, TASK_STATIC);
  TASK_start_timer(led_task, &led_timer1, 1, NULL, 0, 1000, "led1");

  dac_init();
}


#endif /* SRC_APP_C_ */
