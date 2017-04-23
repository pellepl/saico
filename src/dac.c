/*
 * dac.c
 *
 *  Created on: Apr 21, 2017
 *      Author: petera
 */

#include "dac.h"
#include "cli.h"

static DAC_HandleTypeDef  dac_h;
static DAC_ChannelConfTypeDef dac_cfg;
static TIM_HandleTypeDef  tim_h;

static void setup_timer(void) {
  TIM_MasterConfigTypeDef tcfg;

  __HAL_RCC_TIM6_CLK_ENABLE();

  tim_h.Instance = TIM6;
  tim_h.Init.Period            = (SYS_CPU_FREQ / (2) / AUDIO_FREQ) - 1;       // 0..ffff
  tim_h.Init.Prescaler         = 0x0000;       // 0..ffff
  tim_h.Init.ClockDivision     = 0;
  tim_h.Init.CounterMode       = TIM_COUNTERMODE_UP;
  tim_h.Init.RepetitionCounter = 0;
  tim_h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&tim_h);
  tcfg.MasterOutputTrigger = TIM_TRGO_UPDATE;
  tcfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&tim_h, &tcfg);
  HAL_TIM_Base_Start(&tim_h);
}

void dac_init(void) {
  GPIO_InitTypeDef          gpiocfg;
  static DMA_HandleTypeDef  dma_dac_h;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DAC_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  dac_h.Instance = DAC;

  memset(&gpiocfg, 0, sizeof(gpiocfg));
  gpiocfg.Pin = PIN_DAC;
  gpiocfg.Mode = GPIO_MODE_ANALOG;
  gpiocfg.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PORT_DAC, &gpiocfg);

  dma_dac_h.Instance = DMA1_Stream5;
  dma_dac_h.Init.Channel  = DMA_CHANNEL_7;
  dma_dac_h.Init.Direction = DMA_MEMORY_TO_PERIPH;
  dma_dac_h.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_dac_h.Init.MemInc = DMA_MINC_ENABLE;
  dma_dac_h.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  dma_dac_h.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  dma_dac_h.Init.Mode = DMA_CIRCULAR;
  dma_dac_h.Init.Priority = DMA_PRIORITY_MEDIUM;

  HAL_DMA_Init(&dma_dac_h);

  __HAL_LINKDMA(&dac_h, DMA_Handle1, dma_dac_h);

  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  setup_timer();
  ASSERT(HAL_DAC_Init(&dac_h) == HAL_OK);

  print("DAC init\n");
}

void dac_start(u8_t *data, u32_t len) {
  HAL_DAC_DeInit(&dac_h);
  ASSERT(HAL_DAC_Init(&dac_h) == HAL_OK);
  dac_cfg.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  dac_cfg.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  ASSERT(HAL_DAC_ConfigChannel(&dac_h, &dac_cfg, DAC_CHANNEL_1) == HAL_OK);
  ASSERT(HAL_DAC_Start_DMA(&dac_h, DAC_CHANNEL_1, (uint32_t *)data, len, DAC_ALIGN_8B_R) == HAL_OK);
}

void dac_stop(void) {
  HAL_DAC_Stop_DMA(&dac_h, DAC_CHANNEL_1);
}

void dac_sync(void) {
  // this is supposed to trigger the dac dma directly
  tim_h.Instance->CNT = tim_h.Instance->ARR-1;
}

void dac_deinit(void) {
  __HAL_RCC_TIM6_FORCE_RESET();
  __HAL_RCC_TIM6_RELEASE_RESET();
  HAL_DMA_DeInit(dac_h.DMA_Handle1);
  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
}

void DMA1_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(dac_h.DMA_Handle1);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* dac_h) {
  (void)dac_h;
}
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* dac_h) {
  (void)dac_h;
}


u8_t __attribute__ ((aligned (4))) dac_test_data[] = {
    0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff,
};

int cli_dac_test(u32_t argc, u32_t data) {
  dac_start(dac_test_data, sizeof(dac_test_data));
  return CLI_OK;
}

int cli_dac_stop(u32_t argc) {
  dac_stop();
  return CLI_OK;
}

int cli_dac_freq(u32_t argc, u32_t x) {
  u32_t period = (SYS_CPU_FREQ / (2) / x) - 1;
  if (period > 0xffff || period < 1000) {
    print("frequency out of range\n");
    return CLI_OK;
  }
  tim_h.Instance->ARR = period;
  tim_h.Instance->PSC = 0;
  tim_h.Instance->CNT = 0;
  return CLI_OK;
}

int cli_dac_timper(u32_t argc, u32_t x) {
  tim_h.Instance->ARR = x;
  tim_h.Instance->CNT = 0;
  return CLI_OK;
}

int cli_dac_timpsc(u32_t argc, u32_t x) {
  tim_h.Instance->PSC = x;
  tim_h.Instance->CNT = 0;
  return CLI_OK;
}


CLI_MENU_START(dac)
CLI_FUNC("test", cli_dac_test, "send test signal to DAC")
CLI_FUNC("stop", cli_dac_stop, "stop DAC")
CLI_FUNC("freq", cli_dac_freq, "set frequency in Hz (x)")
CLI_FUNC("timper", cli_dac_timper, "set timer period (0-ffff)")
CLI_FUNC("timpsc", cli_dac_timpsc, "set timer prescaler (0-ffff)")
CLI_MENU_END


