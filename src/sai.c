/*
 * sai.c
 *
 *  Created on: Apr 20, 2017
 *      Author: petera
 */

#include "system.h"
#include "sai.h"
#include "miniutils.h"
#include "cli.h"
#include "taskq.h"
#include "dac.h"

#define SAI_A       SAI1_Block_A
#define SAI_B       SAI1_Block_B

#define TX_BUF_SIZE   16*512
#define RX_BUF_SIZE   16*512

SAI_HandleTypeDef sai_h_A;
SAI_HandleTypeDef sai_h_B;
DMA_HandleTypeDef dma_h_A;
DMA_HandleTypeDef dma_h_B;


s16_t tx_buf[TX_BUF_SIZE];
s16_t rx_buf[RX_BUF_SIZE];
static bool txing = FALSE;
static bool dump_rx_raw = FALSE;
static bool analyze_rx_pend = FALSE;
volatile static bool do_dac_sync = FALSE;
static task *analyze_rx_task;
static bool slot_find_activity = FALSE;
static bool slot_show_activity = FALSE;
static u32_t slot_energy[16];

static void analyze_rx_task_fn(u32_t is_half, void *ignore_p);

static void sai_set_master_freq(u32_t target_f_khz, u32_t *actual_f_hz) {
  RCC_PeriphCLKInitTypeDef saiclk;

  const uint32_t multiplier = 1000UL; // As we're running on SAI_CLK on 1MHz
  // 50 <=  PLLSAIN   <= 432
  // 2  <=   PLLSAIQ  <= 15
  // 1  <= PLLSAIDivQ <= 32
  // freq = 1MHz * PLLSAIIN / PLLSAIQ / PLLSAIDivQ
  s32_t min_err = 0x7fffffff;
  u32_t sain, saiq, divq;
  u32_t cand_sain = 0, cand_saiq = 0, cand_divq = 0;
  // this is the unacademic way to solve this - thank god for computers
  for (sain = 50; sain <= 432; sain++) {
    for (saiq = 2; saiq <= 15; saiq++) {
      for (divq = 1; divq <= 32; divq++) {
        u32_t freq = (multiplier * 1000L * sain) / (saiq * divq);
        s32_t err = (s32_t)freq - (s32_t)target_f_khz * 1000L;
        if (ABS(err) < min_err) {
          min_err = ABS(err);
          cand_sain = sain;
          cand_saiq = saiq;
          cand_divq = divq;
        }
      }
    }
  }
  saiclk.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
  saiclk.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  saiclk.PLLSAI.PLLSAIN = cand_sain;
  saiclk.PLLSAI.PLLSAIQ = cand_saiq;
  saiclk.PLLSAIDivQ  = cand_divq;
  ASSERT(HAL_RCCEx_PeriphCLKConfig(&saiclk) == HAL_OK);
  if (actual_f_hz) {
    *actual_f_hz = multiplier * 1000 * cand_sain / (cand_saiq * cand_divq);
  }
}

static void sai_config_default(SAI_HandleTypeDef *sai_h_x, bool tx) {
  __HAL_SAI_DISABLE(sai_h_x);

  sai_h_x->Init.AudioMode = tx ? SAI_MODEMASTER_TX : SAI_MODEMASTER_RX;
  sai_h_x->Init.Synchro = tx ? SAI_SYNCHRONOUS : SAI_ASYNCHRONOUS;
  sai_h_x->Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE; //tx ? SAI_OUTPUTDRIVE_ENABLE : SAI_OUTPUTDRIVE_DISABLE; // TODO
  sai_h_x->Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
  sai_h_x->Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  sai_h_x->Init.AudioFrequency = SAI_AUDIO_FREQUENCY_8K;
  sai_h_x->Init.Protocol = SAI_FREE_PROTOCOL;
  sai_h_x->Init.DataSize = SAI_DATASIZE_16;
  sai_h_x->Init.FirstBit = SAI_FIRSTBIT_MSB;
  sai_h_x->Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;

  sai_h_x->FrameInit.FrameLength = 256;
  sai_h_x->FrameInit.ActiveFrameLength = 16;
  sai_h_x->FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  sai_h_x->FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  sai_h_x->FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  sai_h_x->SlotInit.FirstBitOffset = 0;
  sai_h_x->SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  sai_h_x->SlotInit.SlotNumber = 16;
  sai_h_x->SlotInit.SlotActive = SAI_SLOTACTIVE_ALL;

  ASSERT(HAL_SAI_Init(sai_h_x) == HAL_OK);

  __HAL_SAI_ENABLE(sai_h_x);
}

void sai_init(void) {
  // gpios
  GPIO_InitTypeDef  GPIO_Init;
  GPIO_Init.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init.Pull      = GPIO_NOPULL;
  GPIO_Init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_Init.Pin       = PIN_SAI_A_FS;
  GPIO_Init.Alternate = ALT_SAI_A_FS;
  HAL_GPIO_Init(PORT_SAI_A_FS, &GPIO_Init);
  GPIO_Init.Pin       = PIN_SAI_A_SCK;
  GPIO_Init.Alternate = ALT_SAI_A_SCK;
  HAL_GPIO_Init(PORT_SAI_A_SCK, &GPIO_Init);
  GPIO_Init.Pin       = PIN_SAI_A_SD;
  GPIO_Init.Alternate = ALT_SAI_A_SD;
  HAL_GPIO_Init(PORT_SAI_A_SD, &GPIO_Init);
  GPIO_Init.Pin       = PIN_SAI_A_MCLK;
  GPIO_Init.Alternate = ALT_SAI_A_MCLK;
  HAL_GPIO_Init(PORT_SAI_A_MCLK, &GPIO_Init);

  GPIO_Init.Pin       = PIN_SAI_B_FS;
  GPIO_Init.Alternate = ALT_SAI_B_FS;
  HAL_GPIO_Init(PORT_SAI_B_FS, &GPIO_Init);
  GPIO_Init.Pin       = PIN_SAI_B_SCK;
  GPIO_Init.Alternate = ALT_SAI_B_SCK;
  HAL_GPIO_Init(PORT_SAI_B_SCK, &GPIO_Init);
  GPIO_Init.Pin       = PIN_SAI_B_SD;
  GPIO_Init.Alternate = ALT_SAI_B_SD;
  HAL_GPIO_Init(PORT_SAI_B_SD, &GPIO_Init);
  GPIO_Init.Pin       = PIN_SAI_B_MCLK;
  GPIO_Init.Alternate = ALT_SAI_B_MCLK;
  HAL_GPIO_Init(PORT_SAI_B_MCLK, &GPIO_Init);


  // sai config
  __HAL_RCC_SAI1_CLK_ENABLE();

  sai_set_master_freq(2048, NULL);

  // init SAI
  __HAL_SAI_RESET_HANDLE_STATE(&sai_h_A);

  sai_h_A.Instance = SAI_A;
  sai_h_B.Instance = SAI_B;
  sai_config_default(&sai_h_A, TRUE);
  sai_config_default(&sai_h_B, FALSE);

  __HAL_RCC_DMA2_CLK_ENABLE();

  // SAI1_A: DMA2 Channel0 Stream3
  dma_h_A.Init.Channel             = DMA_CHANNEL_0;
  dma_h_A.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  dma_h_A.Init.PeriphInc           = DMA_PINC_DISABLE;
  dma_h_A.Init.MemInc              = DMA_MINC_ENABLE;
  dma_h_A.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma_h_A.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  dma_h_A.Init.Mode                = DMA_CIRCULAR;
  dma_h_A.Init.Priority            = DMA_PRIORITY_HIGH;
  dma_h_A.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dma_h_A.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_h_A.Init.MemBurst            = DMA_MBURST_SINGLE;
  dma_h_A.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  dma_h_A.Instance                 = DMA2_Stream3;
  __HAL_LINKDMA(&sai_h_A, hdmatx, dma_h_A);
  HAL_DMA_DeInit(&dma_h_A);
  ASSERT(HAL_DMA_Init(&dma_h_A) == HAL_OK);

  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0x01, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  // SAI1_B: DMA2 Channel1 Stream4
  dma_h_B.Init.Channel             = DMA_CHANNEL_1;
  dma_h_B.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  dma_h_B.Init.PeriphInc           = DMA_PINC_DISABLE;
  dma_h_B.Init.MemInc              = DMA_MINC_ENABLE;
  dma_h_B.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma_h_B.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  dma_h_B.Init.Mode                = DMA_CIRCULAR;
  dma_h_B.Init.Priority            = DMA_PRIORITY_HIGH;
  dma_h_B.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dma_h_B.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_h_B.Init.MemBurst            = DMA_MBURST_SINGLE;
  dma_h_B.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  dma_h_B.Instance                 = DMA2_Stream4;
  __HAL_LINKDMA(&sai_h_B, hdmarx, dma_h_B);
  HAL_DMA_DeInit(&dma_h_B);
  ASSERT(HAL_DMA_Init(&dma_h_B) == HAL_OK);

  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0x01, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

  // Other
  int i;
  for (i = 0; i < TX_BUF_SIZE; i++) {
    u8_t v = ((i&0xf) + 1);
    tx_buf[i] = (v == 0x10 ? 0 : v) * 0x1110;
  }
  memset(rx_buf,0x00,sizeof(rx_buf));
  analyze_rx_task = TASK_create(analyze_rx_task_fn, TASK_STATIC);
  analyze_rx_pend = FALSE;
  slot_find_activity = TRUE; // TODO
  slot_show_activity = FALSE;
  memset(slot_energy, 0, sizeof(slot_energy));
}

void DMA2_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(sai_h_A.hdmatx);
}
void DMA2_Stream4_IRQHandler(void) {
  HAL_DMA_IRQHandler(sai_h_B.hdmarx);
}

static void sai_tx_data(void) {
  if (txing) {
    HAL_SAI_DMAStop(&sai_h_A);
    HAL_SAI_DMAStop(&sai_h_B);
  }
  txing = TRUE;
  HAL_SAI_Transmit_DMA(&sai_h_A, (u8_t *)tx_buf, TX_BUF_SIZE);
  HAL_SAI_Receive_DMA(&sai_h_B, (u8_t *)rx_buf, RX_BUF_SIZE);
}

static void analyze_rx_task_fn(u32_t is_half, void *ignore_p) {
  if (dump_rx_raw && is_half) {
    int i;
    for (i = 0; i < 16; i++) {
      print("%04x ", rx_buf[i+16*1]);
    }
    print("\n");
  }
  if (slot_find_activity) {
    s16_t *rx = &rx_buf[is_half ? 0 : RX_BUF_SIZE/2];
    int i, j;
    for (i = 16; i < RX_BUF_SIZE/2; i += 16) {
      for (j = 0; j < 16; j++) {
        s16_t a = rx[i+j];
        s16_t b = rx[i+j-16];
        s16_t d = a-b;
        slot_energy[j] += ABS(d);
      }
    }
    if (is_half == 0) {
      if (slot_show_activity) {
        for (i = 0; i < 16; i++) {
          u32_t e = slot_energy[i] / (RX_BUF_SIZE / 16);
          if (e < 10) {
            print("---- ");
          } else {
            print("%-4x ",  e);
          }
        }
        print("\n");
      }
      memset(slot_energy, 0, sizeof(slot_energy));
    }
  }
  analyze_rx_pend = FALSE;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
  //print("tx\n");
}
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
  //print("tx half\n");
}
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
  //print("rx\n");
  if (do_dac_sync) {
    dac_sync();
    do_dac_sync = FALSE;
  }
  if (!analyze_rx_pend) {
    analyze_rx_pend = TRUE;
    TASK_run(analyze_rx_task, 0, NULL);
  }
}
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
  //print("rx half\n");
  if (!analyze_rx_pend) {
    analyze_rx_pend = TRUE;
    TASK_run(analyze_rx_task, 1, NULL);
  }
}

#define _STRSET(x,y,z) x.y = z
#define SAI_CFG(__what, __set) do {\
    __HAL_SAI_DISABLE(&sai_h_A); __HAL_SAI_DISABLE(&sai_h_B); \
    if (txing) { \
      HAL_SAI_DMAStop(&sai_h_A); \
      HAL_SAI_DMAStop(&sai_h_B); \
    } \
    _STRSET(sai_h_A, __what, __set); \
    _STRSET(sai_h_B, __what, __set); \
    ASSERT(HAL_SAI_Init(&sai_h_A) == HAL_OK);\
    ASSERT(HAL_SAI_Init(&sai_h_B) == HAL_OK);\
    __HAL_SAI_ENABLE(&sai_h_A); __HAL_SAI_ENABLE(&sai_h_B); \
    if (txing) { \
      HAL_SAI_Transmit_DMA(&sai_h_A, (u8_t *)tx_buf, TX_BUF_SIZE); \
      HAL_SAI_Receive_DMA(&sai_h_B, (u8_t *)rx_buf, RX_BUF_SIZE); \
    } \
} while(0)

static void sai_set_audio_freq(u32_t x) {
  SAI_CFG(Init.AudioFrequency, x);
}
static void sai_set_datasize(u32_t x) {
  SAI_CFG(Init.DataSize, x);
}
static void sai_set_firstbit(u32_t x) {
  SAI_CFG(Init.FirstBit, x);
}
static void sai_set_clockstrobe(u32_t x) {
  SAI_CFG(Init.ClockStrobing, x);
}
static void sai_set_frame_length(u32_t x) {
  SAI_CFG(FrameInit.FrameLength, x);
}
static void sai_set_frame_active(u32_t x) {
  SAI_CFG(FrameInit.ActiveFrameLength, x);
}
static void sai_set_frame_def(u32_t x) {
  SAI_CFG(FrameInit.FSDefinition, x);
}
static void sai_set_frame_pol(u32_t x) {
  SAI_CFG(FrameInit.FSPolarity, x);
}
static void sai_set_frame_offs(u32_t x) {
  SAI_CFG(FrameInit.FSOffset, x);
}
static void sai_set_slot_bitoffset(u32_t x) {
  SAI_CFG(SlotInit.FirstBitOffset, x);
}
static void sai_set_slot_size(u32_t x) {
  SAI_CFG(SlotInit.SlotSize, x);
}
static void sai_set_slot_active(u32_t x) {
  SAI_CFG(SlotInit.SlotActive, x);
}
static void sai_set_slot_count(u32_t x) {
  SAI_CFG(SlotInit.SlotNumber, x);
}


static void print_sai_cfg(void) {
  print("--- GENERAL ---\n");
  print("audio freq:    ");
  switch(sai_h_A.Init.AudioFrequency) {
  case SAI_AUDIO_FREQUENCY_8K:     print("8k\n");     break;
  case SAI_AUDIO_FREQUENCY_11K:    print("11k\n");    break;
  case SAI_AUDIO_FREQUENCY_16K:    print("16k\n");    break;
  case SAI_AUDIO_FREQUENCY_22K:    print("22k\n");    break;
  case SAI_AUDIO_FREQUENCY_32K:    print("32k\n");    break;
  case SAI_AUDIO_FREQUENCY_44K:    print("44k\n");    break;
  case SAI_AUDIO_FREQUENCY_48K:    print("48k\n");    break;
  case SAI_AUDIO_FREQUENCY_96K:    print("96k\n");    break;
  case SAI_AUDIO_FREQUENCY_192K:   print("192k\n");   break;
  case SAI_AUDIO_FREQUENCY_MCKDIV: print("mckdiv\n"); break;
  default:                         print("???\n");    break;
  }
  print("datasize:      ");
  switch(sai_h_A.Init.DataSize) {
  case SAI_DATASIZE_8: print("8\n");   break;
  case SAI_DATASIZE_10:print("10\n");  break;
  case SAI_DATASIZE_16:print("16\n");  break;
  case SAI_DATASIZE_20:print("20\n");  break;
  case SAI_DATASIZE_24:print("24\n");  break;
  case SAI_DATASIZE_32:print("32\n");  break;
  default:             print("???\n"); break;
  }
  print("firstbit:      ");
  switch(sai_h_A.Init.FirstBit) {
  case SAI_FIRSTBIT_MSB:print("MSB\n"); break;
  case SAI_FIRSTBIT_LSB:print("LSB\n"); break;
  default:              print("???\n"); break;
  }
  print("clockstrobe:   ");
  switch(sai_h_A.Init.ClockStrobing) {
  case SAI_CLOCKSTROBING_FALLINGEDGE:print("falling edge\n"); break;
  case SAI_CLOCKSTROBING_RISINGEDGE: print("rising edge\n");  break;
  default:                           print("???\n");          break;
  }
  print("--- FRAME ---\n");
  print("total length:  %d\n", sai_h_A.FrameInit.FrameLength);
  print("active length: %d\n", sai_h_A.FrameInit.ActiveFrameLength);
  print("definition:    ");
  switch(sai_h_A.FrameInit.FSDefinition) {
  case SAI_FS_STARTFRAME:             print("start frame\n"); break;
  case SAI_FS_CHANNEL_IDENTIFICATION: print("channel id\n");  break;
  default:                            print("???\n");         break;
  }
  print("polarity:      ");
  switch(sai_h_A.FrameInit.FSPolarity) {
  case SAI_FS_ACTIVE_LOW:  print("low\n");  break;
  case SAI_FS_ACTIVE_HIGH: print("high\n"); break;
  default:                 print("???\n");  break;
  }
  print("offset:        ");
  switch(sai_h_A.FrameInit.FSOffset) {
  case SAI_FS_BEFOREFIRSTBIT: print("b4 1st bit\n"); break;
  case SAI_FS_FIRSTBIT:       print("at 1st bit\n"); break;
  default:                    print("???\n");        break;
  }
  print("--- SLOT ---\n");
  print("1st bitoffset: %d\n", sai_h_A.SlotInit.FirstBitOffset);
  print("slot size:     ");
  switch(sai_h_A.SlotInit.SlotSize) {
  case SAI_SLOTSIZE_DATASIZE: print("data\n"); break;
  case SAI_SLOTSIZE_16B:      print("16b\n");  break;
  case SAI_SLOTSIZE_32B:      print("32b\n");  break;
  default:                    print("???\n");  break;
  }
  print("nbr of slots:  %d\n", sai_h_A.SlotInit.SlotNumber);
  print("active slots:  %016b\n", sai_h_A.SlotInit.SlotActive);
}




//////////////////////////////////////////////////
// CLI stuff
//////////////////////////////////////////////////




static int cli_sai_init(u32_t argc) {
  sai_init();
  return CLI_OK;
}

static int cli_sai_master_freq(u32_t argc, u32_t freq) {
  if (argc != 1) {
    freq = 2048;
  }
  u32_t actual;
  sai_set_master_freq(freq, &actual);
  print("actual frequency %dHz\n", actual);
  return CLI_OK;
}

static int cli_sai_audio_freq(u32_t argc, char *freq) {
  if (argc != 1) {
    freq = "8k";
  }
  if (0 == strcmp("8k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_8K);
  }
  else if (0 == strcmp("11k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_11K);
  }
  else if (0 == strcmp("16k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_16K);
  }
  else if (0 == strcmp("22k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_22K);
  }
  else if (0 == strcmp("32k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_32K);
  }
  else if (0 == strcmp("44k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_44K);
  }
  else if (0 == strcmp("48k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_48K);
  }
  else if (0 == strcmp("96k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_96K);
  }
  else if (0 == strcmp("192k", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_192K);
  }
  else if (0 == strcmp("mck", freq)) {
    sai_set_audio_freq(SAI_AUDIO_FREQUENCY_MCKDIV);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_datasize(u32_t argc, u32_t b) {
  if (argc != 1) {
    b = 8;
  }
  if (b == 8) {
    sai_set_datasize(SAI_DATASIZE_8);
  }
  else if (b == 10) {
    sai_set_datasize(SAI_DATASIZE_10);
  }
  else if (b == 16) {
    sai_set_datasize(SAI_DATASIZE_16);
  }
  else if (b == 20) {
    sai_set_datasize(SAI_DATASIZE_20);
  }
  else if (b == 24) {
    sai_set_datasize(SAI_DATASIZE_24);
  }
  else if (b == 32) {
    sai_set_datasize(SAI_DATASIZE_32);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_firstbit(u32_t argc, char *def) {
  if (argc != 1) {
    def = "msb";
  }
  if (0 == strcmp("msb", def)) {
    sai_set_firstbit(SAI_FIRSTBIT_MSB);
  } else if (0 == strcmp("lsb", def)) {
    sai_set_firstbit(SAI_FIRSTBIT_LSB);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_strobe(u32_t argc, char *def) {
  if (argc != 1) {
    def = "rising";
  }
  if (0 == strcmp("rising", def)) {
    sai_set_clockstrobe(SAI_CLOCKSTROBING_RISINGEDGE);
  } else if (0 == strcmp("falling", def)) {
    sai_set_clockstrobe(SAI_CLOCKSTROBING_FALLINGEDGE);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_frame_length(u32_t argc, u32_t l) {
  if (argc != 1) {
    l = 256;
  }
  if (l < 8 || l > 256) return CLI_ERR_PARAM;
  sai_set_frame_length(l);
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_frame_active(u32_t argc, u32_t l) {
  if (argc != 1) {
    l = 16;
  }
  if (l < 1 || l > 128) return CLI_ERR_PARAM;
  sai_set_frame_active(l);
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_frame_definition(u32_t argc, char *def) {
  if (argc != 1) {
    def = "channel";
  }
  if (0 == strcmp("frame", def)) {
    sai_set_frame_def(SAI_FS_STARTFRAME);
  } else if (0 == strcmp("channel", def)) {
    sai_set_frame_def(SAI_FS_CHANNEL_IDENTIFICATION);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_frame_polarity(u32_t argc, char *def) {
  if (argc != 1) {
    def = "high";
  }
  if (0 == strcmp("high", def)) {
    sai_set_frame_pol(SAI_FS_ACTIVE_LOW);
  } else if (0 == strcmp("low", def)) {
    sai_set_frame_pol(SAI_FS_ACTIVE_HIGH);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_frame_offset(u32_t argc, char *def) {
  if (argc != 1) {
    def = "b4";
  }
  if (0 == strcmp("b4", def)) {
    sai_set_frame_offs(SAI_FS_BEFOREFIRSTBIT);
  } else if (0 == strcmp("at", def)) {
    sai_set_frame_offs(SAI_FS_FIRSTBIT);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_slot_bitoffset(u32_t argc, u32_t l) {
  if (argc != 1) {
    l = 0;
  }
  if (l > 24) return CLI_ERR_PARAM;
  sai_set_slot_bitoffset(l);
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_slot_size(u32_t argc, char *def) {
  if (argc != 1) {
    def = "data";
  }
  if (0 == strcmp("data", def)) {
    sai_set_slot_size(SAI_SLOTSIZE_DATASIZE);
  } else if (0 == strcmp("16b", def)) {
    sai_set_slot_size(SAI_SLOTSIZE_16B);
  } else if (0 == strcmp("32b", def)) {
    sai_set_slot_size(SAI_SLOTSIZE_32B);
  } else {
    return CLI_ERR_PARAM;
  }
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_nbr_of_slots(u32_t argc, u32_t l) {
  if (argc != 1) {
    l = 16;
  }
  if (l < 1 || l > 16) return CLI_ERR_PARAM;
  sai_set_slot_count(l);
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_slot_active(u32_t argc, u32_t l) {
  if (argc != 1) {
    l = 0xffff;
  }
  l &= 0xffff;
  sai_set_slot_active(l);
  print_sai_cfg();
  return CLI_OK;
}

static int cli_sai_tx(u32_t argc) {
  sai_tx_data();
  return CLI_OK;
}

static int cli_sai_dump_rx(u32_t argc, char *x) {
  if (argc != 1) {
    x = "on";
  }
  if (0 == strcmp("on", x)) {
    dump_rx_raw = TRUE;
  } else if (0 == strcmp("off", x)) {
    dump_rx_raw = FALSE;
  } else {
    return CLI_ERR_PARAM;
  }
  return CLI_OK;
}

static int cli_sai_dump_energy(u32_t argc, char *x) {
  if (argc != 1) {
    x = "on";
  }
  if (0 == strcmp("on", x)) {
    slot_show_activity = TRUE;
  } else if (0 == strcmp("off", x)) {
    slot_show_activity = FALSE;
  } else {
    return CLI_ERR_PARAM;
  }
  return CLI_OK;
}

static int cli_sai_dacsync(u32_t argc) {
  do_dac_sync = TRUE;
  return CLI_OK;
}

static int cli_sai_config(u32_t argc) {
  print_sai_cfg();
  return CLI_OK;
}

CLI_MENU_START(sai)
CLI_FUNC("init", cli_sai_init, "initialize SAI")
CLI_FUNC("mfreq", cli_sai_master_freq, "set master clock frequency in kHz")
CLI_FUNC("afreq", cli_sai_audio_freq, "set audio clock frequency (8k|11k|16k|22k|32k|44k|48k|96k|192k|mck)")
CLI_FUNC("datasize", cli_sai_datasize, "set data size bits (8|10|16|20|24|32)")
CLI_FUNC("firstbit", cli_sai_firstbit, "set data firstbit (msb|lsb)")
CLI_FUNC("strobe", cli_sai_strobe, "set clockstrobe (rising|falling)")
CLI_FUNC("framelen", cli_sai_frame_length, "set frame length in bits (8-256)")
CLI_FUNC("frameact", cli_sai_frame_active, "set frame sync length in bits (1-128)")
CLI_FUNC("framedef", cli_sai_frame_definition, "set frame definition (frame|channel)")
CLI_FUNC("framepol", cli_sai_frame_polarity, "set frame polarity (high|low)")
CLI_FUNC("frameoffs", cli_sai_frame_offset, "set frame offset (b4 = before first bit|at = at first bit)")
CLI_FUNC("slotoffs", cli_sai_slot_bitoffset, "set slot first bit offset (0-24)")
CLI_FUNC("slotsize", cli_sai_slot_size, "set slot size (16b|32b|data)")
CLI_FUNC("slotnbr", cli_sai_nbr_of_slots, "set nbr of slots (1-16)")
CLI_FUNC("slotactive", cli_sai_slot_active, "set active slot mask (0bxxxxxxxxxxxxxxxx)")
CLI_FUNC("tx", cli_sai_tx, "start transmit/receive data")
CLI_FUNC("dumprx", cli_sai_dump_rx, "dump parts of raw slot data (on|off)")
CLI_FUNC("dumpe", cli_sai_dump_energy, "dump slot signal energy (on|off)")
CLI_FUNC("dacsync", cli_sai_dacsync, "tries to sync DAC stream")
CLI_FUNC("config", cli_sai_config, "prints SAI config")
CLI_MENU_END

