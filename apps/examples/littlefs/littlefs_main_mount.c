/****************************************************************************
 * apps/examples/littlefs/littlefs_main_mount.c
 *
 * Step 1: SPI Initialization Test
 * Target: STM32WL5 (Nucleo-WL55JC) - SPI1
 *
 ****************************************************************************/

/****************************************************************************
 * Overview
 *
 *   This file talks to the SPI NOR flash on SPI1 (MT25Q family), mounted
 *   as three LittleFS partitions:
 *
 *     /dev/hk1     -> /mnt/hk1     ADC1 housekeeping telemetry (34B/pkt)
 *     /dev/hk2     -> /mnt/hk2     ADC2 + IMU housekeeping telemetry (38B/pkt)
 *     /dev/camera  -> /mnt/camera  Captured camera frames
 *
 *   It has two roles:
 *
 *     1. Low-level flash bring-up / self-test helpers (init_spi,
 *        read_flash_id, read_flash_status, write/erase commands,
 *        littlefs_hardware_test) used to validate the SPI link and the
 *        flash chip before trusting it with real telemetry.
 *
 *     2. The `littlefs_telemetry_daemon`, which is the storage consumer
 *        for OBC_main.c's telemetry producer: it blocks on
 *        g_telemetry_sem, drains packets from the shared ring buffer
 *        (telemetry_rb_read), and appends each one to the matching
 *        LittleFS file, verifying every write with an immediate
 *        read-back.
 *
 *   `littlefs_main` is the NSH-callable entry point: with no arguments it
 *   runs the daemon; with "read"/"clean"/"format" [hk1|hk2|camera] it
 *   performs the corresponding one-shot maintenance operation instead.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>

#include <nuttx/spi/spi.h>
#include <telemetry_rb.h>
#include "ring_buffer.h"
#include "Sring_buffer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_PORT          1          /* Using SPI1                        */
#define SPI_FREQUENCY     20000000   /* 20 MHz                            */

/* MT25Q-family SPI NOR flash opcodes used by the erase helpers below */

#define ERASE_4KB         0x20       /* 4KB Sector Erase command          */
#define ERASE_32KB        0x52       /* 32KB Block Erase command          */
#define ERASE_64KB        0xD8       /* 64KB Block Erase command          */
#define ERASE_ALL         0xC7       /* Chip Erase command (128MB erase)  */
#define Die_erase          0xC4      /* Die Erase command (64MB erase)    */

#define HK_Download_Cmd 53 01 1D D1 F2
/****************************************************************************
 * Forward Declarations
 ****************************************************************************/

/* Board-specific SPI bus initialization (implemented in board.c) */

FAR struct spi_dev_s *stm32wl5_spibus_initialize(int bus);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_spi
 *
 * Description:
 *   Initialize the given SPI bus and configure it for the flash chip:
 *   Mode 0, 8-bit frames, 20 MHz clock.
 ****************************************************************************/

static FAR struct spi_dev_s *init_spi(int bus)
{
  FAR struct spi_dev_s *spi;

  printf("Initializing SPI bus %d...\n", bus);

  /* 1. Initialize the SPI hardware bus */

  spi = stm32wl5_spibus_initialize(bus);
  if (spi == NULL)
    {
      printf("ERROR: stm32wl5_spibus_initialize(%d) returned NULL!\n", bus);
      return NULL;
    }

  /* 2. Configure bus parameters (Mode 0, 8 bits, 20 MHz) */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);        /* Mode 0: CPOL=0, CPHA=0 */
  SPI_SETBITS(spi, 8);                  /* 8-bit data frame */
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, SPI_FREQUENCY);  /* 20 MHz clock */
  SPI_LOCK(spi, false);

  printf("SPI%d initialized and configured successfully.\n", bus);
  return spi;
}

/****************************************************************************
 * Name: read_flash_id
 *
 * Description:
 *   Read and print the MT25Q flash's JEDEC ID (command 0x9F: manufacturer
 *   ID, memory type, capacity code).  Command 0x9E reads the same fields
 *   plus extended data, if ever needed.
 ****************************************************************************/

static int read_flash_id(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x9F; /* Read ID (JEDEC ID) command */
  uint8_t id[3] = {0};

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  /* Send the Read ID command */
  SPI_SEND(spi, cmd);

  /* Receive the ID bytes */
  SPI_RECVBLOCK(spi, id, sizeof(id));

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Manufacture ID: %02X, Memory Type: %02X, Capacity: %02X\n", id[0], id[1], id[2]);
  return OK;
}

/****************************************************************************
 * Name: read_flash_status
 *
 * Description:
 *   Read the flash's Status Register (command 0x05).  Bit 0 (WIP -
 *   "write in progress") is 0 when the flash is idle/ready and 1 while
 *   it is busy erasing or writing.
 ****************************************************************************/

static int read_flash_status(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x05; /* Read Status Register command */
  uint8_t status = 0;

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  /* Send the Read Status Register command */
  SPI_SEND(spi, cmd);

  /* Mode 0 only shifts a byte out in response to a byte sent in, so a
   * dummy byte must be clocked out to receive the status byte back.
   */
  status = SPI_SEND(spi, 0xAA);

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Read Flash Status Register: %02X\n", status);
  return status;
}

/****************************************************************************
 * Name: write_flash_enable
 *
 * Description:
 *   Send the Write Enable command (0x06).  Required before any
 *   program/erase command; the flash clears this latch automatically
 *   after the operation completes.
 ****************************************************************************/

static int write_flash_enable(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x06; /* Write Enable command */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  SPI_SEND(spi, cmd);

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Flash Write Enable command sent.\n");
  return OK;
}

/****************************************************************************
 * Name: write_flash_disable
 *
 * Description:
 *   Send the Write Disable command (0x04), clearing the write-enable
 *   latch set by write_flash_enable().
 ****************************************************************************/

static int write_flash_disable(FAR struct spi_dev_s *spi)
{
  uint8_t cmd = 0x04; /* Write Disable command */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  SPI_SEND(spi, cmd);

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Flash Write Disable command sent.\n");
  return OK;
}

/****************************************************************************
 * Name: erase_flash_4kb
 *
 * Description:
 *   Erase a single 4KB sector at `address` (command 0x20) and poll the
 *   status register until the operation completes or times out
 *   (~5 seconds).
 ****************************************************************************/

static int erase_flash_4kb(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[5];
  uint8_t status;
  int count = 0;
  const int max_retries_4kb = 500; /* 500 tries * 10ms = 5000 ms total */

  cmd[0] = 0x21;                   /* 0x21: 4-Byte 4KB Subsector Erase */
  cmd[1] = (address >> 24) & 0xFF; /* Address byte 1 (MSB) */
  cmd[2] = (address >> 16) & 0xFF; /* Address byte 2 */
  cmd[3] = (address >> 8)  & 0xFF; /* Address byte 3 */
  cmd[4] = address & 0xFF;         /* Address byte 4 (LSB) */

  /* 1. Write Enable MUST be sent first, as its own transaction */
  write_flash_enable(spi);

  /* 2. Send the erase command and address together */
  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);

  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));

  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Flash Erase command sent for address: 0x%08lX\n", (unsigned long)address);

  /* 3. Poll the status register (WIP bit) until the erase finishes */
  printf("Waiting for erase to complete...\n");
  do
    {
      usleep(1000);  /* 1 ms so the CPU can breathe between polls */
      count++;
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (count < max_retries_4kb));

  if (count >= max_retries_4kb)
    {
      printf("RESULT: [FAIL] Erase timed out after %d retries!\n", count);
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 4KB Erase completed in %d iterations (~%d ms)!\n",
         count, count * 10);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_32kb
 *
 * Description:
 *   Erase a single 32KB block at `address` (command 0x52) and poll the
 *   status register until complete or timed out (~1 second).
 ****************************************************************************/

static int erase_flash_32kb(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[5];
  uint8_t status;
  int count = 0;
  const int max_retries_32kb = 1000; /* 1000 * 1ms = 1,000 ms (1 second) */

  cmd[0] = 0x5C;                   /* 0x5C: 4-Byte 32KB Subsector Erase */
  cmd[1] = (address >> 24) & 0xFF;
  cmd[2] = (address >> 16) & 0xFF;
  cmd[3] = (address >> 8)  & 0xFF;
  cmd[4] = address & 0xFF;

  write_flash_enable(spi);

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Erasing 32KB at address 0x%08lX...\n", (unsigned long)address);

  do
    {
      usleep(1000); /* 1ms */
      count++;
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (count < max_retries_32kb));

  if (count >= max_retries_32kb)
    {
      printf("RESULT: [FAIL] 32KB Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 32KB Erased in ~%d ms!\n", count * 10);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_64kb
 *
 * Description:
 *   Erase a single 64KB sector at `address` (command 0xD8) and poll the
 *   status register until complete or timed out (~2 seconds).
 ****************************************************************************/

static int erase_flash_64kb(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[5];
  uint8_t status;
  int count = 0;
  const int max_retries_64kb = 2000; /* 2000 * 1ms = 2,000 ms (2 seconds) */

  cmd[0] = 0xDC;                   /* 0xDC: 4-Byte 64KB Sector Erase */
  cmd[1] = (address >> 24) & 0xFF;
  cmd[2] = (address >> 16) & 0xFF;
  cmd[3] = (address >> 8)  & 0xFF;
  cmd[4] = address & 0xFF;

  write_flash_enable(spi);

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Erasing 64KB sector at address 0x%08lX...\n", (unsigned long)address);

  do
    {
      usleep(10000); /* 10ms */
      count++;
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (count < max_retries_64kb));

  if (count >= max_retries_64kb)
    {
      printf("RESULT: [FAIL] 64KB Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 64KB Erased in ~%d ms!\n", count * 10);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_all
 *
 * Description:
 *   Erase the entire 128MB flash chip (command 0xC7, no address bytes).
 *   Takes several minutes; polls the status register with a heartbeat
 *   dot printed once per second, up to an ~8 minute timeout.
 ****************************************************************************/

static int erase_flash_all(FAR struct spi_dev_s *spi)
{
  uint8_t status;
  int seconds = 0;
  const int max_seconds_all = 500; /* ~8 minutes timeout */

  printf("WARNING: Erasing ENTIRE 128MB Flash Chip (takes ~2-4 minutes)...\n");

  write_flash_enable(spi);

  /* Send ONLY the opcode: chip erase takes no address bytes */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SEND(spi, ERASE_ALL);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  do
    {
      sleep(1); /* 1 second */
      seconds++;
      printf(".");
      fflush(stdout);
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (seconds < max_seconds_all));

  printf("\n");

  if (seconds >= max_seconds_all)
    {
      printf("RESULT: [FAIL] Bulk Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] Entire 128MB Chip Erased in %d seconds!\n", seconds);
  return OK;
}

/****************************************************************************
 * Name: erase_flash_die
 *
 * Description:
 *   Erase a single 64MB die at `address` (command 0xC4).  Polls the
 *   status register with a heartbeat dot printed once per second, up to
 *   an ~5 minute timeout.
 ****************************************************************************/

static int erase_flash_die(FAR struct spi_dev_s *spi, uint32_t address)
{
  uint8_t cmd[5];
  uint8_t status;
  int seconds = 0;
  const int max_seconds_die = 300; /* 5 minutes timeout */

  /* Enter 4-Byte Address Mode (0xB7) so Die Erase (0xC4) accepts 4-byte address */
  write_flash_enable(spi);
  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SEND(spi, 0xB7);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  cmd[0] = Die_erase;              /* 0xC4: Die Erase */
  cmd[1] = (address >> 24) & 0xFF;
  cmd[2] = (address >> 16) & 0xFF;
  cmd[3] = (address >> 8)  & 0xFF;
  cmd[4] = address & 0xFF;

  write_flash_enable(spi);

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), true);
  SPI_SNDBLOCK(spi, cmd, sizeof(cmd));
  SPI_SELECT(spi, SPIDEV_MAIN_FLASH(0), false);
  SPI_LOCK(spi, false);

  printf("Erasing 64MB Die for address 0x%08lX (takes ~1-2 minutes)...\n",
         (unsigned long)address);

  do
    {
      sleep(1); /* 1 second */
      seconds++;
      printf(".");
      fflush(stdout);
      status = read_flash_status(spi);
    }
  while ((status & 0x01) && (seconds < max_seconds_die));

  printf("\n");

  if (seconds >= max_seconds_die)
    {
      printf("RESULT: [FAIL] Die Erase timed out!\n");
      return -ETIMEDOUT;
    }

  printf("RESULT: [PASS] 64MB Die Erased in %d seconds!\n", seconds);
  return OK;
}

/****************************************************************************
 * Name: mount_drive
 *
 * Description:
 *   Mount a LittleFS partition, creating the mount point directory if
 *   needed.  Tries "autoformat" first (mounts an existing filesystem, or
 *   formats only if the partition is blank); if that fails for a reason
 *   other than "already mounted", falls back to "forceformat" to
 *   guarantee a clean, mountable filesystem.
 *
 * Returned Value:
 *   OK on success (including "already mounted"), negative errno on
 *   failure.
 ****************************************************************************/

static int mount_drive_ex(FAR const char *dev_path, FAR const char *mount_path,
                          FAR bool *already_mounted)
{
  struct stat st;
  int ret;

  if (already_mounted != NULL)
    {
      *already_mounted = false;
    }

  printf("Mounting drive %s to %s...\n", dev_path, mount_path);

  /* Ensure parent /mnt mount point exists in VFS */
  mkdir("/mnt", 0777);

  /* 1. Create the mount directory if it doesn't already exist */
  if (stat(mount_path, &st) != 0)
    {
      mkdir(mount_path, 0777);
    }

  /* 2. Try autoformat mount first */
  ret = mount(dev_path, mount_path, "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      int err = errno;
      if (err == EBUSY || err == ENOTDIR || err == EEXIST)
        {
          /* Already mounted: perfectly normal, not an error */
          if (already_mounted != NULL)
            {
              *already_mounted = true;
            }
          return OK;
        }

      if (err == ENOMEM)
        {
          printf("ERROR: mount %s failed: out of memory (ENOMEM)\n", dev_path);
          return -ENOMEM;
        }

      printf("WARN: autoformat failed (errno=%d), trying forceformat...\n", err);

      /* If unformatted, forceformat will format the partition cleanly */
      ret = mount(dev_path, mount_path, "littlefs", 0, "forceformat");
      if (ret < 0)
        {
          err = errno;
          if (err == EBUSY || err == ENOTDIR || err == EEXIST)
            {
              if (already_mounted != NULL)
                {
                  *already_mounted = true;
                }
              return OK;
            }

          printf("ERROR: mount failed (ret=%d, errno=%d)\n", ret, err);
          return -err;
        }
    }

  printf("RESULT: [PASS] Drive %s mounted at %s!\n", dev_path, mount_path);
  return OK;
}

static int mount_drive(FAR const char *dev_path, FAR const char *mount_path)
{
  return mount_drive_ex(dev_path, mount_path, NULL);
}

/****************************************************************************
 * Name: unmount_drive
 *
 * Description:
 *   Safely flush and unmount the LittleFS filesystem at `mount_path`.
 ****************************************************************************/

static int unmount_drive(FAR const char *mount_path)
{
  int ret;

  printf("Unmounting %s...\n", mount_path);

  ret = umount(mount_path);
  if (ret < 0)
    {
      printf("ERROR: umount() failed (ret=%d, errno=%d)\n", ret, errno);
      return ret;
    }

  printf("RESULT: [PASS] Drive at %s safely unmounted!\n", mount_path);
  return OK;
}

static int clean_one_partition(FAR const char *dev_path, FAR const char *mount_path);

/****************************************************************************
 * Name: format_one_partition
 *
 * Description:
 *   Unconditionally reformat one partition ("forceformat"), discarding
 *   any existing contents. If partition is in use by telemetry daemon,
 *   falls back to wiping all files on the partition.
 ****************************************************************************/

static int format_one_partition(FAR const char *dev_path, FAR const char *mount_path)
{
  printf("[LITTLEFS] Formatting %s at %s with 256B page geometry...\n", dev_path, mount_path);
  mkdir("/mnt", 0777);
  umount(mount_path);
  mkdir(mount_path, 0777);

  int ret = mount(dev_path, mount_path, "littlefs", 0, "forceformat");
  if (ret < 0)
    {
      if (errno == EBUSY)
        {
          printf("[LITTLEFS] Partition %s is active (storage daemon running).\n", mount_path);
          printf("[LITTLEFS] Erasing all files on %s instead...\n", mount_path);
          return clean_one_partition(dev_path, mount_path);
        }

      printf("[LITTLEFS] ERROR: Failed to format %s (ret=%d, errno=%d)\n", dev_path, ret, errno);
      return ret;
    }

  /* Unmount immediately so RAM is freed before formatting other partitions */
  umount(mount_path);

  printf("[LITTLEFS] [PASS] %s successfully formatted!\n", dev_path);
  return OK;
}

/****************************************************************************
 * Name: clean_one_partition
 *
 * Description:
 *   Mount one partition (if not already mounted), delete every file in its
 *   root directory without reformatting, and unmount if we mounted it so
 *   heap RAM is not exhausted across 11 partitions.
 ****************************************************************************/

static int clean_one_partition(FAR const char *dev_path, FAR const char *mount_path)
{
  bool already_mounted = false;
  int ret = mount_drive_ex(dev_path, mount_path, &already_mounted);
  if (ret < 0)
    {
      printf("[LITTLEFS] ERROR: Could not mount %s for cleaning (ret=%d)\n",
             dev_path, ret);
      return ret;
    }

  DIR *d = opendir(mount_path);
  if (d)
    {
      struct dirent *de;
      char filepath[64];
      int count = 0;

      while ((de = readdir(d)) != NULL)
        {
          if (strcmp(de->d_name, ".") == 0 || strcmp(de->d_name, "..") == 0)
            {
              continue;
            }

          snprintf(filepath, sizeof(filepath), "%s/%s", mount_path, de->d_name);
          unlink(filepath);
          count++;
        }

      closedir(d);
      printf("[LITTLEFS] [PASS] Cleaned %s: removed %d file(s).\n", mount_path, count);
    }
  else
    {
      int err = errno;
      printf("[LITTLEFS] Could not open %s (errno=%d)\n", mount_path, err);
      if (!already_mounted)
        {
          umount(mount_path);
        }
      return -err;
    }

  if (!already_mounted)
    {
      umount(mount_path);
    }

  return OK;
}

/****************************************************************************
 * Name: write_file
 *
 * Description:
 *   Write `len` bytes from `data` to `filepath`, truncating/creating as
 *   needed, and fsync() before closing so the data is flushed from the
 *   RAM cache to the physical SPI NOR flash.
 ****************************************************************************/

static int write_file(FAR const char *filepath, FAR const void *data, size_t len)
{
  int fd;
  ssize_t nwritten;

  printf("Opening %s to write %zu bytes...\n", filepath, len);

  fd = open(filepath, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0)
    {
      printf("ERROR: open() failed for writing (errno=%d)\n", errno);
      return -errno;
    }

  nwritten = write(fd, data, len);
  if (nwritten != len)
    {
      printf("ERROR: write() failed! (wrote %zd / %zu bytes)\n", nwritten, len);
      close(fd);
      return -EIO;
    }

  fsync(fd);
  close(fd);

  printf("RESULT: [PASS] Successfully wrote %zd bytes to %s!\n", nwritten, filepath);
  return OK;
}

/****************************************************************************
 * Name: read_file
 *
 * Description:
 *   Read up to `max_len` bytes from `filepath` into `buffer`.
 *
 * Returned Value:
 *   Number of bytes read (>= 0) on success, negative errno on failure.
 ****************************************************************************/

static int read_file(FAR const char *filepath, FAR void *buffer, size_t max_len)
{
  int fd;
  ssize_t nread;

  printf("Opening %s to read...\n", filepath);

  fd = open(filepath, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: open() failed for reading (errno=%d)\n", errno);
      return -errno;
    }

  nread = read(fd, buffer, max_len);
  close(fd);

  if (nread < 0)
    {
      printf("ERROR: read() failed (errno=%d)\n", errno);
      return -errno;
    }

  printf("RESULT: [PASS] Successfully read %zd bytes from %s!\n", nread, filepath);
  return (int)nread;
}

/****************************************************************************
 * Name: littlefs_hardware_test
 *
 * Description:
 *   One-shot bench self-test: brings up SPI1, reads the flash JEDEC ID,
 *   exercises the status/write-enable/write-disable commands, then
 *   mounts each die, writes a small test file, reads it back, and
 *   unmounts.  Not part of the normal flight data path; useful when
 *   bringing up new hardware or verifying a flash chip is alive.
 ****************************************************************************/

static int littlefs_hardware_test(void)
{
  FAR struct spi_dev_s *spi;
  int ret;
  uint8_t status_R;
  uint8_t status_W;
  char read_buf[128];

  printf("\n");
  printf("=========================================\n");
  printf(" STEP 1: Test SPI Initialization\n");
  printf("=========================================\n");

  /* Call the SPI initialization function */

  spi = init_spi(SPI_PORT);
  if (spi == NULL)
    {
      printf("RESULT: [FAIL] SPI initialization failed!\n\n");
      return EXIT_FAILURE;
    }

  printf("RESULT: [PASS] SPI%d is ready for communication.\n", SPI_PORT);
  printf("=========================================\n\n");

  ret = read_flash_id(spi);
  if (ret != OK)
    {
      printf("RESULT: [FAIL] Failed to read Flash ID!\n\n");
      return EXIT_FAILURE;
    }
  printf("RESULT: Flash ID read successfully.\n");

  /* Check whether the flash is busy or idle before touching it */

  printf("First Reading Flash Status Register...\n");
  status_R = read_flash_status(spi);
  if (status_R & 0x01)
    {
      printf("RESULT: Flash is busy (Status Register: %02X)\n", status_R);
    }
  else
    {
      printf("RESULT: Flash is ready/idle (Status Register: %02X)\n", status_R);
    }

  /* Exercise the write-enable latch and confirm it toggles as expected */

  printf("Enabling Flash Write Operation...\n");
  write_flash_enable(spi);
  status_W = read_flash_status(spi);
  if (status_W & 0x02)
    {
      printf("RESULT: Enabled (Status Register: %02X)\n", status_W);
    }
  else
    {
      printf("RESULT: Disabled (Status Register: %02X)\n", status_W);
    }

  printf("Disabling Flash Write Operation...\n");
  write_flash_disable(spi);
  status_W = read_flash_status(spi);
  if (status_W & 0x02)
    {
      printf("RESULT: Enabled (Status Register: %02X)\n", status_W);
    }
  else
    {
      printf("RESULT: Disabled (Status Register: %02X)\n", status_W);
    }

  /* Sector-erase test is disabled by default; uncomment to exercise it.
   *
   *   printf("Starting 64KB Sector Erase Test...\n");
   *   ret = erase_flash_64kb(spi, 0x020000);
   *   if (ret != OK)
   *     {
   *       printf("RESULT: [FAIL] 64KB Sector Erase failed!\n\n");
   *       return EXIT_FAILURE;
   *     }
   */

  printf("Starting write dummy data to LittleFS test...\n");
  printf("Mounting LittleFS...\n");

  /* ======================================================================= */
  /* PART 1: TEST DIE 0 (Housekeeping - /dev/hk)                             */
  /* ======================================================================= */

  printf("\n--- Testing Die 0 (Housekeeping) ---\n");

  ret = mount_drive("/dev/hk", "/mnt/hk");
  if (ret != OK)
    {
      printf("Failed to mount /dev/hk!\n");
      return EXIT_FAILURE;
    }

  /* Die 0 write/read-back exercise is currently disabled; uncomment to
   * write and verify a sample telemetry log line, then unmount:
   *
   *   const char *hk_data = "HK_LOG: Batt=4.1V, Temp=22C, SolarCurrent=1.2A, Status=NOMINAL";
   *   write_file("/mnt/hk/telemetry.log", hk_data, strlen(hk_data));
   *   memset(read_buf, 0, sizeof(read_buf));
   *   read_file("/mnt/hk/telemetry.log", read_buf, sizeof(read_buf) - 1);
   *   printf("Die 0 Data Read: \"%s\"\n\n", read_buf);
   *   unmount_drive("/mnt/hk");
   */

  /* ======================================================================= */
  /* PART 2: TEST DIE 1 (Camera - /dev/camera)                               */
  /* ======================================================================= */

  printf("\n--- Testing Die 1 (Camera Images) ---\n");

  /* 1. Mount Die 1 to /mnt/camera */
  ret = mount_drive("/dev/camera", "/mnt/camera");
  if (ret != OK)
    {
      printf("Failed to mount /dev/camera!\n");
      return EXIT_FAILURE;
    }

  /* 2. Write a dummy image file to Die 1 */
  const char *cam_data = "CAMERA_RAW_IMAGE: Width=640, Height=480, Format=RGB565, Frame=1";
  write_file("/mnt/camera/frame001.raw", cam_data, strlen(cam_data));

  /* 3. Read it back from Die 1 */
  memset(read_buf, 0, sizeof(read_buf));
  read_file("/mnt/camera/frame001.raw", read_buf, sizeof(read_buf) - 1);
  printf("Die 1 Data Read: \"%s\"\n\n", read_buf);

  /* 4. Unmount Die 1 */
  unmount_drive("/mnt/camera");

  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: littlefs_telemetry_daemon
 *
 * Description:
 *   Storage consumer for OBC_main's telemetry producer.  Mounts both
 *   housekeeping partitions, then loops forever:
 *
 *     1. Sleep on g_telemetry_sem (0% CPU while idle; posted once per
 *        packet by send_telemetry_to_ringbuffer() in OBC_main).
 *     2. Drain every pending packet from the shared ring buffer.
 *     3. Route each packet by its footer value to the matching file
 *        (ADC1 -> /mnt/hk1/telemetry.bin, ADC2+IMU -> /mnt/hk2/telemetry.bin),
 *        append it, fsync, then read the same bytes back from flash and
 *        compare against what was written as an integrity check.
 *     4. On a write() failure that looks like a stale/old flash format
 *        (EFAULT), automatically force-reformat the partition and retry
 *        the append once.
 ****************************************************************************/

static int littlefs_telemetry_daemon(void)
{
  /* Run the unified single-ring-buffer, single-active-mount storage consumer
   * for all 11 MT25Q partitions (HK1, HK2, ADCS, CAM_RGB, CAM_NIR, EPDM,
   * DPM, RSV_Table, Imp_Flags, Part_Size, Seek_Ptr).
   */
  return storage_consumer_run();
}

/****************************************************************************
 * Name: littlefs_read_hk1
 *
 * Description:
 *   Mount /dev/hk1 if needed and print every stored ADC1 telemetry
 *   record in /mnt/hk1/telemetry.bin.
 ****************************************************************************/

static int littlefs_read_hk1(void)
{
  uint8_t rec[128];
  static const char * const names[ADC1_CHANNELS_COUNT] =
  {
    "ADC_BAT_MON", "TOTAL_SOLAR_V", "RAW_VOLT", "SP5_VOLT",
    "SP4_VOLT",    "SP3_VOLT",      "SP1_VOLT", "SP2_VOLT",
    "ANT_TEMP",    "BATT_TEMP",     "TEMP_BPB", "TEMP1",
    "TEMP5",       "TEMP4",         "TEMP3",    "TEMP2"
  };

  /* Auto-mount /dev/hk if not already mounted */
  mount_drive("/dev/hk", "/mnt/hk");

  int fd = open("/mnt/hk/telemetry.bin", O_RDONLY);
  if (fd < 0)
    {
      if (errno == ENOENT)
        {
          printf("[INFO] /mnt/hk/telemetry.bin does not exist yet.\n");
          printf("[INFO] Please run 'launcher' first to record telemetry packets!\n");
        }
      else
        {
          printf("ERROR: Cannot open /mnt/hk/telemetry.bin (errno=%d)\n", errno);
        }
      return -errno;
    }

  uint32_t count = 0;
  ssize_t nread;
  printf("\n======================= READING /mnt/hk/telemetry.bin (HK1) =======================\n");
  while ((nread = read(fd, rec, sizeof(rec))) == sizeof(rec))
    {
      count++;
      int16_t *adc1 = (int16_t *)&rec[0];
      uint16_t footer1 = (uint16_t)(rec[32] | ((uint16_t)rec[33] << 8));
      printf("\n>>> HK1 Telemetry Record #%lu (128B, Footer: 0x%04X) <<<\n",
             (unsigned long)count, footer1);
      for (int i = 0; i < ADC1_CHANNELS_COUNT; i++)
        {
          const char *unit = (i < 8) ? "V" : "C";
          printf("  [%02d] %-15s : %6.2f %s   (Raw x100: %6d)\n",
                 i, names[i], (float)adc1[i] / 100.0f, unit, adc1[i]);
        }
    }

  close(fd);
  printf("\n[TOTAL] Read %lu HK1 records from /mnt/hk/telemetry.bin\n", (unsigned long)count);
  return 0;
}

/****************************************************************************
 * Name: littlefs_read_hk2
 *
 * Description:
 *   Mount /dev/hk2 if needed and print every stored ADC2+IMU telemetry
 *   record in /mnt/hk2/telemetry.bin.
 ****************************************************************************/

static int littlefs_read_hk2(void)
{
  uint8_t rec[128];
  static const char * const names[ADC2_ACTIVE_CHANNELS] =
  {
    "UNREG_I",       "MAIN_3V3_I",    "5V_I",        "BAT_I",
    "SP1_I",         "SP2_I",         "SP3_I",       "SP4_I",
    "SP5_I",         "RAW_I",         "MISSION_3V3", "SPT_I"
  };

  /* Auto-mount /dev/hk if not already mounted */
  mount_drive("/dev/hk", "/mnt/hk");

  int fd = open("/mnt/hk/telemetry.bin", O_RDONLY);
  if (fd < 0)
    {
      if (errno == ENOENT)
        {
          printf("[INFO] /mnt/hk/telemetry.bin does not exist yet.\n");
          printf("[INFO] Please run 'launcher' first to record telemetry packets!\n");
        }
      else
        {
          printf("ERROR: Cannot open /mnt/hk/telemetry.bin (errno=%d)\n", errno);
        }
      return -errno;
    }

  uint32_t count = 0;
  ssize_t nread;
  printf("\n======================= READING /mnt/hk/telemetry.bin (HK2) =======================\n");
  while ((nread = read(fd, rec, sizeof(rec))) == sizeof(rec))
    {
      count++;
      int16_t *adc2 = (int16_t *)&rec[34];
      int16_t *imu  = (int16_t *)&rec[58];
      uint16_t footer2 = (uint16_t)(rec[70] | ((uint16_t)rec[71] << 8));
      printf("\n>>> HK2 Telemetry Record #%lu (128B, Footer: 0x%04X) <<<\n",
             (unsigned long)count, footer2);
      for (int i = 0; i < ADC2_ACTIVE_CHANNELS; i++)
        {
          printf("  [%02d] %-15s : %6.2f A   (Raw x100: %6d)\n",
                 i, names[i], (float)adc2[i] / 100.0f, adc2[i]);
        }
      printf("  [GYRO] X: %6.2f dps | Y: %6.2f dps | Z: %6.2f dps   (x100: %d, %d, %d)\n",
             (float)imu[0] / 100.0f, (float)imu[1] / 100.0f, (float)imu[2] / 100.0f,
             imu[0], imu[1], imu[2]);
      printf("  [MAG]  X: %6.3f G   | Y: %6.3f G   | Z: %6.3f G   (x100: %d, %d, %d)\n",
             (float)imu[3] / 100.0f, (float)imu[4] / 100.0f, (float)imu[5] / 100.0f,
             imu[3], imu[4], imu[5]);
    }

  close(fd);
  printf("\n[TOTAL] Read %lu HK2 records from /mnt/hk/telemetry.bin\n", (unsigned long)count);
  return 0;
}
/*sends the HK1, HK2 and IMU data brust if command received*/
static void __attribute__((unused)) send_brust_HK(uint32_t start_address, uint16_t packet_count)
{
  (void)start_address;
  (void)packet_count;
}

struct lfs_part_info_s
{
  FAR const char *key;
  FAR const char *alias;
  FAR const char *label;
  FAR const char *dev_path;
  FAR const char *mnt_path;
  FAR const char *default_file;
  uint32_t        size_mb;
};

static const struct lfs_part_info_s g_lfs_parts[] =
{
  { "hk1",       NULL,    "HK_1 (ADC1 & ADC2)",      "/dev/hk1",       "/mnt/hk1",       "/mnt/hk1/telemetry.bin",      32 },
  { "hk2",       NULL,    "HK_2 (Mag & Gyro)",       "/dev/hk2",       "/mnt/hk2",       "/mnt/hk2/imu.bin",            32 },
  { "adcs",      NULL,    "ADCS MSN",                "/dev/adcs",      "/mnt/adcs",      "/mnt/adcs/adcs.bin",           8 },
  { "cam_rgb",   "rgb",   "CAM RGB MSN",             "/dev/cam_rgb",   "/mnt/cam_rgb",   "/mnt/cam_rgb/capture_01.raw", 16 },
  { "cam_nir",   "nir",   "CAM NIR MSN",             "/dev/cam_nir",   "/mnt/cam_nir",   "/mnt/cam_nir/capture_02.raw", 16 },
  { "epdm",      NULL,    "EPDM MSN",                "/dev/epdm",      "/mnt/epdm",      "/mnt/epdm/epdm.bin",           8 },
  { "dpm",       NULL,    "DP-M MSN",                "/dev/dpm",       "/mnt/dpm",       "/mnt/dpm/dpm.bin",             8 },
  { "rsv_table", "rsv",   "RSV_Table",               "/dev/rsv_table", "/mnt/rsv_table", "/mnt/rsv_table/rsv.bin",       2 },
  { "imp_flags", "flags", "Important Flags",         "/dev/imp_flags", "/mnt/imp_flags", "/mnt/imp_flags/flags.bin",     2 },
  { "part_size", "size",  "All Partition File Size", "/dev/part_size", "/mnt/part_size", "/mnt/part_size/size.bin",      2 },
  { "seek_ptr",  "seek",  "Seek_Pointer",            "/dev/seek_ptr",  "/mnt/seek_ptr",  "/mnt/seek_ptr/seek.bin",       2 },
};

#define LFS_NUM_PARTS (sizeof(g_lfs_parts) / sizeof(g_lfs_parts[0]))

/****************************************************************************
 * Name: apply_partition_op
 *
 * Description:
 *   Apply `op` (clean_one_partition or format_one_partition) to the target
 *   partition(s): "all" (or NULL), "hk" (hk1+hk2), "camera"/"cam"
 *   (cam_rgb+cam_nir), or any individual partition key/alias.
 ****************************************************************************/

static int apply_partition_op(FAR const char *target,
                              int (*op)(FAR const char *, FAR const char *),
                              FAR const char *op_name)
{
  int ret = OK;

  if (target == NULL || strcasecmp(target, "all") == 0)
    {
      printf("[LITTLEFS] %s all %zu flash partitions...\n", op_name, LFS_NUM_PARTS);
      for (size_t i = 0; i < LFS_NUM_PARTS; i++)
        {
          int r = op(g_lfs_parts[i].dev_path, g_lfs_parts[i].mnt_path);
          if (r < 0)
            {
              ret = r;
            }
        }
      return ret;
    }

  if (strcasecmp(target, "hk") == 0)
    {
      printf("[LITTLEFS] %s HK partitions (hk1, hk2)...\n", op_name);
      int r1 = op("/dev/hk1", "/mnt/hk1");
      int r2 = op("/dev/hk2", "/mnt/hk2");
      return (r1 < 0) ? r1 : r2;
    }

  if (strcasecmp(target, "camera") == 0 || strcasecmp(target, "cam") == 0)
    {
      printf("[LITTLEFS] %s Camera partitions (cam_rgb, cam_nir)...\n", op_name);
      int r1 = op("/dev/cam_rgb", "/mnt/cam_rgb");
      int r2 = op("/dev/cam_nir", "/mnt/cam_nir");
      return (r1 < 0) ? r1 : r2;
    }

  for (size_t i = 0; i < LFS_NUM_PARTS; i++)
    {
      if (strcasecmp(target, g_lfs_parts[i].key) == 0 ||
          (g_lfs_parts[i].alias != NULL && strcasecmp(target, g_lfs_parts[i].alias) == 0))
        {
          return op(g_lfs_parts[i].dev_path, g_lfs_parts[i].mnt_path);
        }
    }

  printf("[LITTLEFS] Unknown partition target: '%s'\n", target);
  printf("Valid targets: all | hk | hk1 | hk2 | adcs | camera | cam_rgb | cam_nir | "
         "epdm | dpm | rsv_table | imp_flags | part_size | seek_ptr\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: print_partition_sizes
 *
 * Description:
 *   Display MT25Q NOR flash partition table, capacity, and current file sizes.
 ****************************************************************************/

static int print_partition_sizes(void)
{
  struct stat st;
  printf("\n=================================================================\n");
  printf("  MT25Q SPI NOR FLASH PARTITION TABLE & CURRENT SIZES\n");
  printf("  Total Flash Capacity: 1 Gbit / 128 MB (Die 0: 64MB, Die 1: 64MB)\n");
  printf("=================================================================\n");

  for (size_t i = 0; i < LFS_NUM_PARTS; i++)
    {
      bool already_mounted = false;
      int mret = mount_drive_ex(g_lfs_parts[i].dev_path, g_lfs_parts[i].mnt_path,
                                &already_mounted);
      printf("  [%2zu] %-14s (%2lu MB) -> %-14s (%s)\n",
             i + 1,
             g_lfs_parts[i].dev_path,
             (unsigned long)g_lfs_parts[i].size_mb,
             g_lfs_parts[i].mnt_path,
             g_lfs_parts[i].label);

      if (mret == OK && stat(g_lfs_parts[i].default_file, &st) == 0)
        {
          printf("       File: %s | Size: %ld Bytes\n",
                 g_lfs_parts[i].default_file, (long)st.st_size);
        }
      else
        {
          printf("       File: %s | Size: 0 Bytes (empty)\n",
                 g_lfs_parts[i].default_file);
        }

      if (mret == OK && !already_mounted)
        {
          umount(g_lfs_parts[i].mnt_path);
        }
    }

  printf("=================================================================\n\n");
  return 0;
}

/****************************************************************************
 * Name: littlefs_erase_cmd
 *
 * Description:
 *   Erase partition filesystem or trigger low-level flash sector/block erase.
 ****************************************************************************/

static int littlefs_erase_cmd(int argc, FAR char *argv[])
{
  if (argc < 3)
    {
      int ret = apply_partition_op("all", format_one_partition, "Formatting");
      printf("\n[TIP] You can also target specific partitions or raw flash:\n");
      printf("  littlefs erase hk|hk1|hk2|camera|cam_rgb|cam_nir|adcs|epdm|dpm|rsv_table|imp_flags|part_size|seek_ptr\n");
      printf("  littlefs erase all            - Full 128MB hardware chip erase\n");
      printf("  littlefs erase die [addr]     - 64MB hardware die erase (0x00000000 / 0x04000000)\n\n");
      return ret;
    }

  /* Hardware-level raw SPI NOR erase commands */
  if (strcmp(argv[2], "all") == 0 || strcmp(argv[2], "chip") == 0 ||
      strcmp(argv[2], "die") == 0 || strcmp(argv[2], "64kb") == 0 ||
      strcmp(argv[2], "32kb") == 0 || strcmp(argv[2], "4kb") == 0)
    {
      FAR struct spi_dev_s *spi = init_spi(SPI_PORT);
      if (spi == NULL)
        {
          printf("ERROR: Failed to initialize SPI%d for flash erase!\n", SPI_PORT);
          return -EIO;
        }

      if (strcmp(argv[2], "all") == 0 || strcmp(argv[2], "chip") == 0)
        {
          return erase_flash_all(spi);
        }
      else if (strcmp(argv[2], "die") == 0)
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_die(spi, addr);
        }
      else if (strcmp(argv[2], "64kb") == 0)
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_64kb(spi, addr);
        }
      else if (strcmp(argv[2], "32kb") == 0)
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_32kb(spi, addr);
        }
      else
        {
          uint32_t addr = 0;
          if (argc > 3) addr = (uint32_t)strtoul(argv[3], NULL, 0);
          return erase_flash_4kb(spi, addr);
        }
    }

  /* Partition-level LittleFS forceformat */
  return apply_partition_op(argv[2], format_one_partition, "Formatting");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: littlefs_main
 *
 * Description:
 *   NSH entry point.
 *
 *     littlefs                       -> run telemetry storage daemon
 *     littlefs read   [hk1|hk2]      -> dump stored telemetry records
 *     littlefs clean  [target|all]   -> delete files, keep format
 *     littlefs format [target|all]   -> force-reformat LittleFS partition(s)
 *     littlefs erase  [target|all|chip|die|4kb|32kb|64kb] -> flash erase
 *     littlefs size   [partition/info] -> show partition table & file sizes
 *     littlefs test                  -> run low-level SPI NOR self-test
 ****************************************************************************/

int littlefs_main(int argc, FAR char *argv[])
{
  if (argc > 1)
    {
      if (strcmp(argv[1], "read") == 0)
        {
          if (argc > 2 && strcmp(argv[2], "hk1") == 0)
            {
              return littlefs_read_hk1();
            }
          else if (argc > 2 && strcmp(argv[2], "hk2") == 0)
            {
              return littlefs_read_hk2();
            }
          else
            {
              littlefs_read_hk1();
              littlefs_read_hk2();
              return 0;
            }
        }
      else if (strcmp(argv[1], "clean") == 0 || strcmp(argv[1], "clear") == 0)
        {
          return apply_partition_op((argc > 2) ? argv[2] : "all",
                                    clean_one_partition, "Cleaning");
        }
      else if (strcmp(argv[1], "format") == 0)
        {
          return apply_partition_op((argc > 2) ? argv[2] : "all",
                                    format_one_partition, "Formatting");
        }
      else if (strcmp(argv[1], "size") == 0 || strcmp(argv[1], "partition") == 0 ||
               strcmp(argv[1], "part") == 0 || strcmp(argv[1], "info") == 0)
        {
          return print_partition_sizes();
        }
      else if (strcmp(argv[1], "erase") == 0)
        {
          return littlefs_erase_cmd(argc, argv);
        }
      else if (strcmp(argv[1], "test") == 0)
        {
          return littlefs_hardware_test();
        }
      else if (strcmp(argv[1], "help") == 0 || strcmp(argv[1], "-h") == 0)
        {
          printf("\nUsage: littlefs [command] [target]\n");
          printf("Commands:\n");
          printf("  read   [hk | hk1 | hk2]       - Dump stored telemetry records\n");
          printf("  clean  [target | all]         - Delete files, keep format\n");
          printf("  format [target | all]         - Force-reformat LittleFS partition(s)\n");
          printf("  erase  [target | 4kb | 32kb | 64kb | die | all] - Flash erase\n");
          printf("  size   [or partition/info]    - Display partition table & file sizes\n");
          printf("  test                          - Run low-level SPI NOR self-test\n");
          printf("  (no args)                     - Run telemetry storage daemon\n");
          printf("Targets:\n");
          printf("  all, hk (hk1+hk2), hk1, hk2, adcs, camera (cam_rgb+cam_nir),\n");
          printf("  cam_rgb (rgb), cam_nir (nir), epdm, dpm, rsv_table (rsv),\n");
          printf("  imp_flags (flags), part_size (size), seek_ptr (seek)\n\n");
          return 0;
        }
      else
        {
          printf("[LITTLEFS] Unknown command: '%s'. Run 'littlefs help' for usage.\n",
                 argv[1]);
          return -EINVAL;
        }
    }

  return littlefs_telemetry_daemon();
}