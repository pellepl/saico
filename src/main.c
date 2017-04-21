#include "system.h"
#include "miniutils.h"
#include "taskq.h"
#include "cli.h"
#include "app.h"
#include "uart_driver.h"

static void clock_config(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

volatile bool handling_uart = FALSE;
static uint8_t rxbuf[256];
static void uart_rx_task(uint32_t i, void *i_p) {
  int len;
  while ((len = IO_get_buf(IOSTD, rxbuf, sizeof(rxbuf))) > 0) {
    cli_recv((char *) rxbuf, len);
  }
  handling_uart = FALSE;
}
static void uart_rx_int_f(u8_t c, void *u, u16_t len) {
  if (!handling_uart) {
    handling_uart = TRUE;
    task *uart_task = TASK_create(uart_rx_task, 0);
    TASK_run(uart_task, 0, NULL);
  }
}

static void setup(void) {
  HAL_Init();
  // clock init
  clock_config();

  GPIO_InitTypeDef gpio_conf;

  RCC_OscInitTypeDef osc;
  RCC_ClkInitTypeDef clock;
  uint32_t fLatency;
  memset(&osc, 0, sizeof(osc));
  memset(&clock, 0, sizeof(clock));
  HAL_RCC_GetOscConfig(&osc);
  HAL_RCC_GetClockConfig(&clock, &fLatency);

  HAL_SYSTICK_Config(SystemCoreClock / SYS_MAIN_TIMER_FREQ);
  HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0U);

  // gpio led
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio_conf.Pin = LED_PIN;
  gpio_conf.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_conf.Pull = GPIO_NOPULL;
  gpio_conf.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_conf.Alternate = 0;
  HAL_GPIO_Init(LED_PORT, &gpio_conf);

  // uart
  __GPIOD_CLK_ENABLE();
  gpio_conf.Pin       = GPIO_PIN_5;
  gpio_conf.Mode      = GPIO_MODE_AF_PP;
  gpio_conf.Pull      = GPIO_PULLUP;
  gpio_conf.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_conf.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &gpio_conf);
  gpio_conf.Pin = GPIO_PIN_6;
  gpio_conf.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &gpio_conf);
  NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, 4, 0U);

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  RCC_PeriphClkInit.Usart1ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  __USART2_CLK_ENABLE();
  UART_init();

  //IO_blocking_tx(IOSTD, TRUE);
  IO_assure_tx(IOSTD, TRUE);
  IO_set_callback(IOSTD, uart_rx_int_f, NULL);


  print("\n==================================\n");
  print("BSP information\n");
  print("==================================\n");
  print("HSE state:      %d\n", osc.HSEState);
  print("HSI state:      %d\n", osc.HSIState);
  print("LSE state:      %d\n", osc.LSEState);
  print("LSI state:      %d\n", osc.LSIState);
  print("HSI calibration:%d\n", osc.HSICalibrationValue);
  print("OSC type:       %d\n", osc.OscillatorType);
  print("PLL M:          %d\n", osc.PLL.PLLM);
  print("PLL N:          %d\n", osc.PLL.PLLN);
  print("PLL P:          %d\n", osc.PLL.PLLP);
  print("PLL Q:          %d\n", osc.PLL.PLLQ);
  print("PLL source:     %d\n", osc.PLL.PLLSource);
  print("PLL state:      %d\n", osc.PLL.PLLState);
  print("AHB div:        %d\n", clock.AHBCLKDivider);
  print("APB1 div:       %d\n", clock.APB1CLKDivider);
  print("APB2 div:       %d\n", clock.APB2CLKDivider);
  print("Clock type:     %d\n", clock.ClockType);
  print("SYSCLK source:  %d\n", clock.SYSCLKSource);
  print("f latency:      %d\n", fLatency);
  print("HCLK freq:      %d\n", HAL_RCC_GetHCLKFreq());

  TASK_init();

  cli_init();

  app_init();

  while (1) {
    while (TASK_tick())
      ;
    __WFI();
  }
}

int main(void) {
  SCB_EnableICache();
  SCB_EnableDCache();
  __HAL_FLASH_ART_ENABLE();
  setup();
  return 0;
}

void SysTick_Handler(void) {
  HAL_IncTick();
  if (SYS_timer()) {
    TASK_timer();
  }
}
#if CONFIG_CLI_SYS_OFF == 0
CLI_EXTERN_MENU(system)
#endif
CLI_EXTERN_MENU(sai)

CLI_MENU_START_MAIN
CLI_SUBMENU(sai, "sai", "SAI submenu")
#if CONFIG_CLI_SYS_OFF == 0
CLI_SUBMENU(system, "sys", "SYSTEM submenu")
# endif
CLI_FUNC("help", cli_help, "lists commands")
CLI_MENU_END

