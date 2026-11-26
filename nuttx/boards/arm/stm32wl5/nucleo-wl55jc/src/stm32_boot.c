/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>

#include <nuttx/debug.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/input/buttons.h>

#include <arch/board/board.h>

#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_VIDEO_FB
#include <nuttx/video/fb.h>
#endif

#include <stm32wl5.h>
#include <stm32wl5_uart.h>
#include <stm32wl5_pwr.h>

#include "arm_internal.h"
#include "nucleo-wl55jc.h"

#if defined(CONFIG_ADC_ADS7953)
#include <nuttx/analog/ads7953.h>
#endif

#if defined(CONFIG_STM32WL5_SPI2S2)
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define proc mountpoint in case procfs is used but nsh is not */

#ifndef CONFIG_NSH_PROC_MOUNTPOINT
#define CONFIG_NSH_PROC_MOUNTPOINT "/proc"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_board_initialize
 *
 * Description:
 *   All STM32WL5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/
#ifdef CONFIG_MTD_M25P

static int stm32wl5_mt25q_initialize(void)
{
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  uint8_t id[3] = {0};

  syslog(LOG_INFO, "Initializing external MT25Q NOR Flash...\n");

  /* Initialize SPI1 */

  spi = stm32wl5_spibus_initialize(1);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: SPI1 initialization failed\n");
      return -ENODEV;
    }

  /* Query JEDEC ID directly over SPI1 (RDID 0x9F) */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SEND(spi, 0x9f);
  id[0] = SPI_SEND(spi, 0xff); /* Manufacturer ID (0x20 = Micron) */
  id[1] = SPI_SEND(spi, 0xff); /* Memory Type (0xBA = MT25QL 3V, 0xBB = MT25QU 1.8V) */
  id[2] = SPI_SEND(spi, 0xff); /* Capacity (0x21 = 1 Gbit / 128 MB) */
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  syslog(LOG_INFO, "MT25Q JEDEC ID: Mfg=0x%02X, Type=0x%02X, Cap=0x%02X\n",
         id[0], id[1], id[2]);

  if (id[0] == 0x20 && (id[1] == 0xba || id[1] == 0xbb) && id[2] == 0x21)
    {
      syslog(LOG_INFO, "Detected Micron MT25QL01G (1 Gbit / 128 MB)\n");
    }
  else
    {
      syslog(LOG_WARNING,
             "WARNING: Unexpected JEDEC ID (check SPI wiring/power)\n");
    }

  /* Bind SPI1 to M25P/MT25Q driver */

  mtd = m25p_initialize(spi);
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: m25p_initialize() failed\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "MT25Q MTD initialized successfully\n");

  /* Now initialize the MTD partitions (128 MB MT25QL01G) */
#ifdef CONFIG_MTD_PARTITION
  /* MT25Q Page size is 256 bytes (1 MTD block = 256 bytes) */
  #define MTD_BLOCK_SIZE      256U

  static const struct
  {
    const char *dev_path;
    const char *label;
    uint32_t    start_addr;
    uint32_t    size_bytes;
  } g_mt25q_parts[] =
  {
    { "/dev/hk1",       "HK_1 (ADC1 & ADC2)",    0x00000000U, 32U * 1024U * 1024U },
    { "/dev/hk2",       "HK_2 (Mag & Gyro)",     0x02000000U, 32U * 1024U * 1024U },
    { "/dev/adcs",      "ADCS MSN",              0x04000000U,  8U * 1024U * 1024U },
    { "/dev/cam_rgb",   "CAM RGB MSN",           0x04800000U, 16U * 1024U * 1024U },
    { "/dev/cam_nir",   "CAM NIR MSN",           0x05800000U, 16U * 1024U * 1024U },
    { "/dev/epdm",      "EPDM MSN",              0x06800000U,  8U * 1024U * 1024U },
    { "/dev/dpm",       "DP-M MSN",              0x07000000U,  8U * 1024U * 1024U },
    { "/dev/rsv_table", "RSV_Table",             0x07800000U,  2U * 1024U * 1024U },
    { "/dev/imp_flags", "Important Flags",       0x07A00000U,  2U * 1024U * 1024U },
    { "/dev/part_size", "All Partition File Size", 0x07C00000U, 2U * 1024U * 1024U },
    { "/dev/seek_ptr",  "Seek_Pointer",          0x07E00000U,  2U * 1024U * 1024U },
  };

  printf("MT25Q Flash Partitions (Total 128 MB):\n");
  for (size_t i = 0; i < sizeof(g_mt25q_parts) / sizeof(g_mt25q_parts[0]); i++)
    {
      uint32_t start_blk = g_mt25q_parts[i].start_addr / MTD_BLOCK_SIZE;
      uint32_t num_blks  = g_mt25q_parts[i].size_bytes / MTD_BLOCK_SIZE;
      struct mtd_dev_s *part = mtd_partition(mtd, start_blk, num_blks);
      if (part != NULL)
        {
          register_mtddriver(g_mt25q_parts[i].dev_path, part, 0666, NULL);
          syslog(LOG_INFO, "Registered %s (%s, %lu MB @ 0x%08lX)\n",
                 g_mt25q_parts[i].dev_path, g_mt25q_parts[i].label,
                 (unsigned long)(g_mt25q_parts[i].size_bytes / (1024U * 1024U)),
                 (unsigned long)g_mt25q_parts[i].start_addr);
        }
      else
        {
          syslog(LOG_ERR, "ERROR: mtd_partition failed for %s\n",
                 g_mt25q_parts[i].dev_path);
        }

      printf("  %-14s %-24s Addr=0x%08lX Blk=%6lu..%6lu (%2lu MB)\n",
             g_mt25q_parts[i].dev_path, g_mt25q_parts[i].label,
             (unsigned long)g_mt25q_parts[i].start_addr,
             (unsigned long)start_blk,
             (unsigned long)(start_blk + num_blks - 1U),
             (unsigned long)(g_mt25q_parts[i].size_bytes / (1024U * 1024U)));
    }
#endif

  return OK;
}

#endif

/* Initialized true to match GPIO_OUTPUT_SET on PB8 in
 * stm32wl5_board_initialize(), so the first kick produces a real edge.
 */

static bool g_ext_wdt_state = true;

static inline void stm32wl5_ext_wdt_kick(void)
{
  g_ext_wdt_state = !g_ext_wdt_state;

  if (g_ext_wdt_state)
    {
      putreg32(1UL << 8, 0x48000418UL);       /* PB8 HIGH */
    }
  else
    {
      putreg32(1UL << 24, 0x48000418UL);      /* PB8 LOW */
    }
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
static int stm32wl5_ext_wdt_task(int argc, char *argv[])
{
  UNUSED(argc);
  UNUSED(argv);

  while (1)
    {
      stm32wl5_ext_wdt_kick();

      nxsig_usleep(500000);  /* 500 ms */
    }

  return OK;
}
#endif

void stm32wl5_board_initialize(void)
{
  /* External 1.6 s Watchdog WDI (PB8): push-pull output, kick immediately
   * from bootup (edge-triggered: must toggle faster than 1.6 s).
   * Global Reset pin PA11: drive LOW (idle) at bootup; pulsed HIGH every 24h.
   */

  modifyreg32(0x5800004c, 0, (1UL << 0) | (1UL << 1)); /* RCC_AHB2ENR: GPIOA, GPIOB */
  modifyreg32(0x5800014c, 0, (1UL << 0) | (1UL << 1)); /* RCC_C2AHB2ENR: GPIOA, GPIOB */

  stm32wl5_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz |
                      GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN8);
  stm32wl5_ext_wdt_kick();

  stm32wl5_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz |
                      GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN11);
  stm32wl5_gpiowrite(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz |
                     GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN11, false);

  /* Configure on-board LEDs, which are always enabled */

  board_leds_initialize();
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_BOARD_ENABLE_CPU2)
/****************************************************************************
 * Name: stm32wl5_ensure_cpu2_boot_vector
 *
 * Description:
 *   Ensures that the silicon Option Bytes (FLASH_SRRVR) point CPU2
 *   (Cortex-M0+) to 0x08032000 in Flash. If unconfigured or at factory
 *   default, unlocks Option Bytes, writes the correct boot vector, and
 *   triggers an Option Byte reload (system reset).
 *
 ****************************************************************************/

static void stm32wl5_ensure_cpu2_boot_vector(void)
{
  /* SBRV for 0x08032000:
   * (0x08032000 - 0x08000000) >> 2 = 0x32000 >> 2 = 0xC800.
   * Bit 31 (C2OPT) = 1 (Boot from Flash).
   */

  const uint32_t target_sbrv  = (0x00032000UL >> 2); /* 0xC800 */
  const uint32_t target_srrvr = (1UL << 31) | target_sbrv;
  uint32_t srrvr;

  srrvr = getreg32(STM32WL5_FLASH_SRRVR);

  /* Check if SBRV and C2OPT are already properly set */

  if ((srrvr & 0x8000FFFFUL) == target_srrvr)
    {
      return; /* Hardware already points CPU2 to 0x08032000 */
    }

  printf("\n[SYSTEM] [AUTO-CONFIG] CPU2 boot vector mismatch! (Current SRRVR: 0x%08lx, Target: 0x%08lx)\n",
         (unsigned long)srrvr, (unsigned long)target_srrvr);
  printf("[SYSTEM] [AUTO-CONFIG] Automatically configuring silicon Option Bytes to 0x08032000...\n");
  fflush(stdout);

  /* 1. Wait for any active flash operation to finish */

  while (getreg32(STM32WL5_FLASH_SR) & (FLASH_SR_BSY | FLASH_SR_CFGBSY))
    {
      up_udelay(10);
    }

  /* Drop sticky option-byte errors. A set OPTVERR makes OPTSTRT a no-op,
   * and launching OBL anyway resets forever with the old vector.
   */

  putreg32(FLASH_SR_OPTVERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR |
           FLASH_SR_OPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR |
           FLASH_SR_SIZERR, STM32WL5_FLASH_SR);

  /* 2. Unlock Flash Control Register if locked */

  if (getreg32(STM32WL5_FLASH_CR) & FLASH_CR_LOCK)
    {
      putreg32(0x45670123UL, STM32WL5_FLASH_KEYR);
      putreg32(0xCDEF89ABUL, STM32WL5_FLASH_KEYR);
    }

  /* 3. Unlock Option Bytes if locked */

  if (getreg32(STM32WL5_FLASH_CR) & FLASH_CR_OPTLOCK)
    {
      putreg32(0x08192A3BUL, STM32WL5_FLASH_OPTKEYR);
      putreg32(0x4C5D6E7FUL, STM32WL5_FLASH_OPTKEYR);
    }

  /* 4. Write new CPU2 boot vector into FLASH_SRRVR.
   *    Only C2OPT and SBRV change. nBOOT0 / RDP in FLASH_OPTR stay as they are.
   */

  modifyreg32(STM32WL5_FLASH_SRRVR, 0x8000FFFFUL, target_srrvr);

  /* 5. Start Option Byte modification */

  modifyreg32(STM32WL5_FLASH_CR, 0, FLASH_CR_OPTSTRT);

  /* 6. Wait until modification finishes */

  while (getreg32(STM32WL5_FLASH_SR) & (FLASH_SR_BSY | FLASH_SR_CFGBSY))
    {
      up_udelay(10);
    }

  srrvr = getreg32(STM32WL5_FLASH_SR);
  if ((srrvr & (FLASH_SR_OPTVERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR |
                FLASH_SR_OPERR)) != 0 ||
      (getreg32(STM32WL5_FLASH_SRRVR) & 0x8000FFFFUL) != target_srrvr)
    {
      printf("[SYSTEM] [AUTO-CONFIG] Option byte write failed (SR=0x%08lx). "
             "M4 continues without reset.\n",
             (unsigned long)srrvr);
      fflush(stdout);
      return;
    }

  printf("[SYSTEM] [AUTO-CONFIG] Option Bytes programmed! Reloading OBL (resetting system)...\n");
  fflush(stdout);
  up_mdelay(100);

  /* 7. Launch Option Byte reload (forces hardware reset to apply new boot address) */

  modifyreg32(STM32WL5_FLASH_CR, 0, FLASH_CR_OBL_LAUNCH);

  /* If the reload does not reset the chip, do not spin here. A stuck
   * loop at this point looks like "M4 NuttX does nothing" after flash.
   */

  up_mdelay(500);
  printf("[SYSTEM] [AUTO-CONFIG] OBL reload did not reset. Continuing boot.\n");
  fflush(stdout);
}

static bool stm32wl5_cpu2_image_present(void)
{
  const uint32_t sp    = getreg32(0x08032000);
  const uint32_t reset = getreg32(0x08032004);

  /* Blank CPU2 pages are 0xFFFFFFFF. Booting that image locks the bus
   * and the M4 never reaches the shell.
   */

  if (sp == 0xffffffffu || reset == 0xffffffffu)
    {
      return false;
    }

  /* Stack must point at SRAM1/SRAM2, reset at the CPU2 flash window, Thumb. */

  if ((sp & 0xfffe0000u) != 0x20000000u)
    {
      return false;
    }

  if ((reset & 1u) == 0u || reset < 0x08032000u || reset >= 0x08040000u)
    {
      return false;
    }

  return true;
}
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

  /* Start dedicated external 1.6 s watchdog task (500 ms PB8 toggle).
   * Must be created here (not in stm32wl5_board_initialize()) because
   * the NuttX heap, scheduler, and system tick are only active after
   * nx_start(). Uses 1024-byte stack since CONFIG_ARCH_INTERRUPTSTACK=0.
   */

  ret = kthread_create("ext_wdt",
                       SCHED_PRIORITY_MAX - 10,
                       1024,
                       stm32wl5_ext_wdt_task,
                       NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start ext_wdt kthread: %d\n", ret);
    }

#if defined(CONFIG_ARCH_BOARD_ENABLE_CPU2)
  /* Pre-enable CPU2 peripheral clocks (IPCC, FLASH, HSEM, GPIOA/B/C) early
   * so CPU2 never HardFaults on unclocked GPIO/IPCC if already active on reset.
   */
  modifyreg32(0x58000150, 0, (1 << 20) | (1 << 25) | (1 << 19)); /* C2AHB3ENR */
  modifyreg32(0x5800014c, 0, (1 << 0) | (1 << 1) | (1 << 2));   /* C2AHB2ENR */
#endif

#if defined(CONFIG_STM32WL5_SPI1) || defined(CONFIG_STM32WL5_SPI2S2)
  stm32wl5_spidev_initialize();
#endif

#if defined(CONFIG_LCD_SSD1680) && !defined(CONFIG_VIDEO_FB)
  board_lcd_initialize();
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d\n",
             ret);
      return;
    }
#endif

#if defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_INPUT_BUTTONS_LOWER)
  /* Register the Button driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_FLASH_MOUNT)
  /* Register partition table for on-board FLASH memory */

  ret = stm32wl5_flash_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32wl5_flash_init() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_IPCC) && 0
  /* Register generic IPCC driver - DISABLED:
   * M4 and M0+ use dedicated hardware doorbells via Sring_buffer.h and shared SRAM2
   * ring buffer at 0x20008000. Enabling generic character driver attaches interrupts
   * that conflict with and corrupt shared SRAM2 ring buffer memory.
   */

  ret = ipcc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ipcc_init() failed\n");
    }
#endif

#if defined(CONFIG_MTD_M25P)
  /* Initialize external MT25Q NOR flash and partitions */

  ret = stm32wl5_mt25q_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32wl5_mt25q_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_ENABLE_CPU2)
  /* Ensure Option Bytes point CPU2 to 0x08032000 (Self-configuring) */

  stm32wl5_ensure_cpu2_boot_vector();

  /* Start second CPU (Cortex-M0+) */

  /* Ensure CPU2 peripheral clocks for Flash, IPCC, HSEM, and GPIOA/B/C are pre-enabled */
  modifyreg32(0x58000150, 0, (1 << 20) | (1 << 25) | (1 << 19)); /* C2AHB3ENR: IPCCEN | FLASHEN | HSEMEN */
  modifyreg32(0x5800014c, 0, (1 << 0) | (1 << 1) | (1 << 2));   /* C2AHB2ENR: GPIOA/B/C */

  if (!stm32wl5_cpu2_image_present())
    {
      printf("[SYSTEM] CPU2 flash at 0x08032000 is empty or invalid. "
             "M4 continues, CPU2 not started.\n");
      fflush(stdout);
    }
  else
    {
      printf("[SYSTEM] Booting Cortex-M0+ (CPU2) at 0x08032000 (M4: 200KB, M0+: 56KB)...\n");
      fflush(stdout);
      up_mdelay(10);
      stm32wl5_pwr_boot_c2();
    }

  /* Perform immediate initial IPCC Handshake with Cortex-M0+ */
  modifyreg32(0x58000050, 0, (1 << 20)); /* RCC_AHB3ENR: ensure IPCC clock enabled on M4 */
  putreg32((1 << 0), 0x58000C08);        /* IPCC_C1SCR: Clear channel 1 RX flag */
  int boot_hs_retries = 30;
  bool boot_hs_ok = false;
  while (boot_hs_retries-- > 0)
    {
      putreg32((1 << 16), 0x58000C08);   /* IPCC_C1SCR: Set channel 1 TX flag to M0+ */
      up_mdelay(10);
      if ((getreg32(0x58000C1C) & (1 << 0)) != 0) /* IPCC_C2TOC1SR: Check if M0+ answered */
        {
          putreg32((1 << 0), 0x58000C08); /* IPCC_C1SCR: Clear RX flag */
          boot_hs_ok = true;
          break;
        }
    }
  if (boot_hs_ok)
    {
      printf("[SYSTEM] Cortex-M0+ IPCC Handshake SUCCESSFUL on boot!\n");
    }
  else
    {
      printf("[SYSTEM] Cortex-M0+ boot started (Handshake pending)...\n");
    }
#endif

#if defined(CONFIG_STM32WL5_SPI2S2) && defined(CONFIG_ADC_ADS7953)
  /* Initialize ADS7953 ADC1 and ADC2 on SPI2 */

  if (g_spi2 != NULL)
    {
      ret = ads7953_register("/dev/adc0", g_spi2, SPIDEV_USER(0));
      if (ret < 0)
        {
          printf("ERROR: ads7953_register(/dev/adc0) failed: %d\n", ret);
        }
      else
        {
          printf("ADS7953 ADC1 registered at /dev/adc0\n");
        }

      ret = ads7953_register("/dev/adc1", g_spi2, SPIDEV_USER(1));
      if (ret < 0)
        {
          printf("ERROR: ads7953_register(/dev/adc1) failed: %d\n", ret);
        }
      else
        {
          printf("ADS7953 ADC2 registered at /dev/adc1\n");
        }
    }
  else
    {
      printf("ERROR: g_spi2 is NULL\n");
    }
#endif

  UNUSED(ret);
}
#endif

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  return -ENOTTY;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32wl5_get_uniqueid(uniqueid);
  return OK;
}
#endif
