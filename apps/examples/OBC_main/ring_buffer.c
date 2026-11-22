/****************************************************************************
 * apps/examples/OBC_main/ring_buffer.c
 *
 * Unified Single Ring Buffer + Semaphore + Single-Active-Mount LittleFS
 * Storage Manager for all 11 MT25Q external flash partitions.
 ****************************************************************************/

#include "ring_buffer.h"

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <nuttx/mutex.h>

/* ========================================================================= */
/* Partition Table (Matches stm32_boot.c 11 MT25Q MTD Partitions)            */
/* ========================================================================= */

struct storage_part_desc_s
{
  const char *name;
  const char *dev_path;
  const char *mnt_path;
  const char *file_path;
};

static const struct storage_part_desc_s g_storage_parts[STORAGE_PART_COUNT] =
{
  [STORAGE_PART_HK1]       = { "HK1",       "/dev/hk1",       "/mnt/hk1",       "/mnt/hk1/telemetry.bin"      },
  [STORAGE_PART_HK2]       = { "HK2",       "/dev/hk2",       "/mnt/hk2",       "/mnt/hk2/imu.bin"            },
  [STORAGE_PART_ADCS]      = { "ADCS",      "/dev/adcs",      "/mnt/adcs",      "/mnt/adcs/adcs.bin"          },
  [STORAGE_PART_CAM_RGB]   = { "CAM_RGB",   "/dev/cam_rgb",   "/mnt/cam_rgb",   "/mnt/cam_rgb/capture_01.raw" },
  [STORAGE_PART_CAM_NIR]   = { "CAM_NIR",   "/dev/cam_nir",   "/mnt/cam_nir",   "/mnt/cam_nir/capture_02.raw" },
  [STORAGE_PART_EPDM]      = { "EPDM",      "/dev/epdm",      "/mnt/epdm",      "/mnt/epdm/epdm.bin"          },
  [STORAGE_PART_DPM]       = { "DPM",       "/dev/dpm",       "/mnt/dpm",       "/mnt/dpm/dpm.bin"            },
  [STORAGE_PART_RSV_TABLE] = { "RSV_TABLE", "/dev/rsv_table", "/mnt/rsv_table", "/mnt/rsv_table/rsv.bin"      },
  [STORAGE_PART_IMP_FLAGS] = { "IMP_FLAGS", "/dev/imp_flags", "/mnt/imp_flags", "/mnt/imp_flags/flags.bin"    },
  [STORAGE_PART_PART_SIZE] = { "PART_SIZE", "/dev/part_size", "/mnt/part_size", "/mnt/part_size/size.bin"     },
  [STORAGE_PART_SEEK_PTR]  = { "SEEK_PTR",  "/dev/seek_ptr",  "/mnt/seek_ptr",  "/mnt/seek_ptr/seek.bin"      },
};

/* ========================================================================= */
/* ONE Unified Ring Buffer + Two Semaphores (Items Ready / Free Slots)       */
/* ========================================================================= */

struct storage_rb_s
{
  struct storage_item_s slots[STORAGE_RB_DEPTH];
  volatile uint32_t     wr_ptr;
  volatile uint32_t     rd_ptr;
  volatile uint32_t     count;
};

static struct storage_rb_s g_storage_rb;
static mutex_t             g_storage_lock      = NXMUTEX_INITIALIZER;
static sem_t               g_storage_items_sem; /* Wakes consumer when data ready */
static sem_t               g_storage_space_sem; /* Backpressure when queue full   */
static bool                g_storage_inited    = false;
static volatile bool       g_consumer_running  = false;

/* Single-active-mount LittleFS state (ONLY 1 partition mounted at a time) */

static mutex_t             g_flash_io_lock     = NXMUTEX_INITIALIZER;
static uint8_t             g_active_part       = STORAGE_PART_NONE;
static int                 g_active_fd         = -1;
static size_t              g_active_written    = 0;

/* Exported legacy symbols required by apps/include/telemetry_rb.h */

struct telemetry_rb_s g_telemetry_rb;
sem_t                 g_telemetry_sem;

/* ========================================================================= */
/* Internal Single-Mount Flash Helpers (caller holds g_flash_io_lock)        */
/* ========================================================================= */

static void storage_close_and_unmount_locked(void)
{
  if (g_active_fd >= 0)
    {
      fsync(g_active_fd);
      close(g_active_fd);
      g_active_fd = -1;
    }

  if (g_active_part < STORAGE_PART_COUNT)
    {
      umount(g_storage_parts[g_active_part].mnt_path);
      g_active_part = STORAGE_PART_NONE;
      g_active_written = 0;
    }
}

static int storage_mount_locked(uint8_t part)
{
  if (part >= STORAGE_PART_COUNT)
    {
      return -EINVAL;
    }

  if (g_active_part == part)
    {
      return 0;
    }

  /* Unmount any previously active partition FIRST so only 1 mount exists in RAM */

  storage_close_and_unmount_locked();

  const struct storage_part_desc_s *p = &g_storage_parts[part];
  struct stat st;

  mkdir("/mnt", 0777);
  if (stat(p->mnt_path, &st) != 0)
    {
      mkdir(p->mnt_path, 0777);
    }

  int ret = mount(p->dev_path, p->mnt_path, "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      int err = errno;
      if (err == EBUSY || err == EEXIST || err == ENOTDIR)
        {
          g_active_part = part;
          return 0;
        }

      if (err == ENOMEM)
        {
          struct mallinfo mi = mallinfo();
          printf("[STORAGE] ERROR: mount(%s) ENOMEM (heap free=%d B, largest=%d B)\n",
                 p->dev_path, mi.fordblks, mi.mxordblk);
          return -ENOMEM;
        }

      printf("[STORAGE] WARN: autoformat %s failed (errno=%d), trying forceformat...\n",
             p->dev_path, err);
      ret = mount(p->dev_path, p->mnt_path, "littlefs", 0, "forceformat");
      if (ret < 0)
        {
          err = errno;
          if (err != EBUSY && err != EEXIST && err != ENOTDIR)
            {
              printf("[STORAGE] ERROR: forceformat %s failed (errno=%d)\n",
                     p->dev_path, err);
              return -err;
            }
        }
    }

  g_active_part = part;
  g_active_written = 0;
  return 0;
}

static void storage_write_item_locked(const struct storage_item_s *item)
{
  if (item == NULL || item->part >= STORAGE_PART_COUNT)
    {
      return;
    }

  const struct storage_part_desc_s *p = &g_storage_parts[item->part];

  if (storage_mount_locked(item->part) < 0)
    {
      return;
    }

  /* Truncate / open new file when requested */

  if ((item->flags & STORAGE_FLAG_TRUNC) != 0U)
    {
      if (g_active_fd >= 0)
        {
          fsync(g_active_fd);
          close(g_active_fd);
          g_active_fd = -1;
        }

      g_active_written = 0;
      g_active_fd = open(p->file_path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
      if (g_active_fd < 0)
        {
          /* Recover from corrupt FS by forceformatting once */
          umount(p->mnt_path);
          g_active_part = STORAGE_PART_NONE;
          if (mount(p->dev_path, p->mnt_path, "littlefs", 0, "forceformat") == 0)
            {
              g_active_part = item->part;
              g_active_fd = open(p->file_path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
            }
        }

      if (g_active_fd < 0)
        {
          printf("[STORAGE] ERROR: open(%s, TRUNC) failed (errno=%d)\n",
                 p->file_path, errno);
          storage_close_and_unmount_locked();
          return;
        }
    }
  else if (g_active_fd < 0)
    {
      g_active_fd = open(p->file_path, O_WRONLY | O_CREAT | O_APPEND, 0666);
      if (g_active_fd < 0)
        {
          umount(p->mnt_path);
          g_active_part = STORAGE_PART_NONE;
          if (mount(p->dev_path, p->mnt_path, "littlefs", 0, "forceformat") == 0)
            {
              g_active_part = item->part;
              g_active_fd = open(p->file_path, O_WRONLY | O_CREAT | O_APPEND, 0666);
            }
        }

      if (g_active_fd < 0)
        {
          printf("[STORAGE] ERROR: open(%s, APPEND) failed (errno=%d)\n",
                 p->file_path, errno);
          storage_close_and_unmount_locked();
          return;
        }
    }

  /* Write payload bytes */

  if (item->len > 0 && g_active_fd >= 0)
    {
      ssize_t w = write(g_active_fd, item->data, item->len);
      if (w > 0)
        {
          g_active_written += (size_t)w;
          if (item->part == STORAGE_PART_ADCS ||
              (g_active_written % 2048U) < (size_t)w)
            {
              printf("[STORAGE %s] Stored +%zd B -> %s (total=%u B)\n",
                     p->name, w, p->file_path, (unsigned)g_active_written);
            }
        }
      else
        {
          printf("[STORAGE] ERROR: write(%s, %u B) failed (errno=%d)\n",
                 p->file_path, (unsigned)item->len, errno);
        }
    }

  /* Finalize stream: fsync, close, log size, and unmount to free RAM */

  if ((item->flags & STORAGE_FLAG_FINAL) != 0U)
    {
      if (g_active_fd >= 0)
        {
          fsync(g_active_fd);
          close(g_active_fd);
          g_active_fd = -1;
        }

      struct stat st;
      long disk_sz = (stat(p->file_path, &st) == 0) ? (long)st.st_size : -1L;
      if (disk_sz > 0)
        {
          printf("[STORAGE %s] SUCCESSFULLY STORED in external flash -> %s (verified file size: %ld B)\n",
                 p->name, p->file_path, disk_sz);
        }
      else
        {
          printf("[STORAGE %s] FAILED to store data in external flash -> %s (written=%u B, file size=%ld B)\n",
                 p->name, p->file_path, (unsigned)g_active_written, disk_sz);
        }

      storage_close_and_unmount_locked();
    }
}

/* Pop one item non-blocking (internal helper) */

static int storage_rb_pop_nowait(struct storage_item_s *out)
{
  nxmutex_lock(&g_storage_lock);
  if (g_storage_rb.count == 0)
    {
      nxmutex_unlock(&g_storage_lock);
      return -1;
    }

  uint32_t slot = g_storage_rb.rd_ptr & (STORAGE_RB_DEPTH - 1U);
  *out = g_storage_rb.slots[slot];
  g_storage_rb.rd_ptr++;
  g_storage_rb.count--;
  nxmutex_unlock(&g_storage_lock);

  sem_post(&g_storage_space_sem);
  return 0;
}

/* ========================================================================= */
/* Unified Ring Buffer Public API                                            */
/* ========================================================================= */

void storage_rb_init(void)
{
  if (!g_storage_inited)
    {
      memset(&g_storage_rb, 0, sizeof(g_storage_rb));
      sem_init(&g_storage_items_sem, 0, 0);
      sem_init(&g_storage_space_sem, 0, STORAGE_RB_DEPTH);
      sem_init(&g_telemetry_sem, 0, 0);
      g_storage_inited = true;
    }
}

int storage_rb_push(storage_part_t part, uint8_t flags,
                    const void *data, uint16_t len)
{
  if (part >= STORAGE_PART_COUNT || len > STORAGE_CHUNK_SIZE)
    {
      return -EINVAL;
    }

  storage_rb_init();

  /* If no background consumer is running and buffer is full, drain 1 slot
   * synchronously so we never block or lose data in standalone mode.
   */
  if (!g_consumer_running)
    {
      if (sem_trywait(&g_storage_space_sem) < 0)
        {
          struct storage_item_s tmp;
          if (storage_rb_pop_nowait(&tmp) == 0)
            {
              nxmutex_lock(&g_flash_io_lock);
              storage_write_item_locked(&tmp);
              nxmutex_unlock(&g_flash_io_lock);
            }
          if (sem_trywait(&g_storage_space_sem) < 0)
            {
              return -EAGAIN;
            }
        }
    }
  else
    {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += 2;
      if (sem_timedwait(&g_storage_space_sem, &ts) < 0)
        {
          return -ETIMEDOUT;
        }
    }

  nxmutex_lock(&g_storage_lock);
  uint32_t slot = g_storage_rb.wr_ptr & (STORAGE_RB_DEPTH - 1U);
  g_storage_rb.slots[slot].part  = (uint8_t)part;
  g_storage_rb.slots[slot].flags = flags;
  g_storage_rb.slots[slot].len   = len;
  if (len > 0 && data != NULL)
    {
      memcpy(g_storage_rb.slots[slot].data, data, len);
    }
  g_storage_rb.wr_ptr++;
  g_storage_rb.count++;
  nxmutex_unlock(&g_storage_lock);

  /* Notify single storage consumer via semaphore */

  sem_post(&g_storage_items_sem);
  return 0;
}

int storage_rb_count(void)
{
  if (!g_storage_inited)
    {
      return 0;
    }

  nxmutex_lock(&g_storage_lock);
  int n = (int)g_storage_rb.count;
  nxmutex_unlock(&g_storage_lock);
  return n;
}

void storage_rb_flush(void)
{
  storage_rb_init();

  if (g_consumer_running)
    {
      /* Wake consumer and wait until ring buffer is empty */
      sem_post(&g_storage_items_sem);
      int retries = 500; /* up to 5 seconds */
      while (storage_rb_count() > 0 && retries-- > 0)
        {
          usleep(10000);
        }
    }
  else
    {
      /* Standalone mode: drain queue synchronously */
      struct storage_item_s item;
      nxmutex_lock(&g_flash_io_lock);
      while (storage_rb_pop_nowait(&item) == 0)
        {
          storage_write_item_locked(&item);
        }
      nxmutex_unlock(&g_flash_io_lock);
    }

  /* Guarantee 0 active LittleFS mounts before caller opens/reads flash */

  nxmutex_lock(&g_flash_io_lock);
  storage_close_and_unmount_locked();
  nxmutex_unlock(&g_flash_io_lock);
}

int storage_consumer_run(void)
{
  struct storage_item_s item;

  storage_rb_init();
  g_consumer_running = true;

  printf("\n=======================================================\n");
  printf("  [STORAGE DAEMON] Unified Ring Buffer Consumer Active\n");
  printf("  Buffer : %d slots x %d B (%d B RAM)\n",
         STORAGE_RB_DEPTH, STORAGE_CHUNK_SIZE,
         STORAGE_RB_DEPTH * STORAGE_CHUNK_SIZE);
  printf("  Policy : Single active LittleFS mount (all 11 parts)\n");
  printf("=======================================================\n");

  for (;;)
    {
      /* Sleep on semaphore (0% CPU). If idle for 5 seconds (longer than the
       * 4 s EPDM silence timeout), unmount active partition so heap stays free.
       */
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += 5;

      int wret = sem_timedwait(&g_storage_items_sem, &ts);
      if (wret < 0 && errno == ETIMEDOUT)
        {
          if (storage_rb_count() == 0)
            {
              nxmutex_lock(&g_flash_io_lock);
              if (g_active_part != STORAGE_PART_NONE)
                {
                  storage_close_and_unmount_locked();
                }
              nxmutex_unlock(&g_flash_io_lock);
            }
          continue;
        }

      /* Drain all queued items */

      while (storage_rb_pop_nowait(&item) == 0)
        {
          nxmutex_lock(&g_flash_io_lock);
          storage_write_item_locked(&item);
          nxmutex_unlock(&g_flash_io_lock);
        }
    }

  g_consumer_running = false;
  return 0;
}

/* ========================================================================= */
/* Legacy Camera Ring Buffer Wrappers -> Unified Ring Buffer                 */
/* ========================================================================= */

void rb_init(void)
{
  storage_rb_init();
}

int rb_write(const struct image_chunk_s *c)
{
  if (c == NULL)
    {
      return -1;
    }

  storage_part_t part = (c->frame_id == 2) ? STORAGE_PART_CAM_NIR
                                           : STORAGE_PART_CAM_RGB;
  uint8_t flags = STORAGE_FLAG_APPEND;
  uint16_t len  = c->len;

  if (c->is_meta)
    {
      /* Metadata marker signals start of a new image file */
      flags |= STORAGE_FLAG_TRUNC;
      len    = 0;
    }

  if (c->is_last)
    {
      flags |= STORAGE_FLAG_FINAL;
    }

  return (storage_rb_push(part, flags, c->data, len) == 0) ? 0 : -1;
}

int rb_read(struct image_chunk_s *c)
{
  (void)c;
  return -1; /* Drained directly by unified storage consumer */
}

int rb_count(void)
{
  return storage_rb_count();
}

int rb_full(void)
{
  return (storage_rb_count() >= STORAGE_RB_DEPTH) ? 1 : 0;
}

int rb_empty(void)
{
  return (storage_rb_count() == 0) ? 1 : 0;
}

int ensure_camera_partition_mounted(void)
{
  /* Partitions are mounted on demand (1 at a time) to preserve SRAM */
  return 0;
}

void unmount_camera_partitions(void)
{
  storage_rb_flush();
  umount("/mnt/cam_rgb");
  umount("/mnt/cam_nir");
}

void cam_rb_drain_to_flash(void)
{
  storage_rb_flush();
}

void cam_rb_close_all_files(void)
{
  storage_rb_flush();
}

/* ========================================================================= */
/* Legacy HK Telemetry Wrappers -> Unified Ring Buffer (HK1 + HK2)           */
/* ========================================================================= */

static struct adc1_packet_s g_cached_adc1;
static bool                 g_have_adc1    = false;
static uint32_t             g_hk_flash_seq = 0;

static uint16_t hk_flash_crc16(const uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < len; i++)
    {
      crc ^= ((uint16_t)buf[i] << 8);
      for (uint8_t b = 0; b < 8; b++)
        {
          crc = (crc & 0x8000U) ? ((crc << 1) ^ 0x1021U) : (crc << 1);
        }
    }
  return crc;
}

void telemetry_rb_init(void)
{
  storage_rb_init();
}

int telemetry_rb_write(const struct telemetry_envelope_s *data)
{
  if (data == NULL)
    {
      return -1;
    }

  storage_rb_init();

  if (data->footer == TELEM_FOOTER_ADC1)
    {
      g_cached_adc1 = data->pkt.adc1;
      g_have_adc1   = true;
      return 0;
    }

  if (data->footer == TELEM_FOOTER_ADC2)
    {
      uint8_t hk128[128];
      memset(hk128, 0, sizeof(hk128));

      if (g_have_adc1)
        {
          memcpy(&hk128[0], g_cached_adc1.data, 32);
        }
      hk128[32] = 0x55;
      hk128[33] = 0xAA;

      memcpy(&hk128[34], data->pkt.adc2.data, 24);
      memcpy(&hk128[58], &data->pkt.adc2.gyro_x, 12);
      hk128[70] = 0x66;
      hk128[71] = 0xBB;

      g_hk_flash_seq++;
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      uint32_t now_ms = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

      memcpy(&hk128[72], &g_hk_flash_seq, 4);
      memcpy(&hk128[76], &now_ms, 4);

      uint16_t crc = hk_flash_crc16(hk128, 124);
      hk128[124] = (uint8_t)(crc & 0xFF);
      hk128[125] = (uint8_t)(crc >> 8);
      hk128[126] = 0xAA;
      hk128[127] = 0xCC;

      /* Push HK1 record into unified ring buffer */
      storage_rb_push(STORAGE_PART_HK1, STORAGE_FLAG_APPEND | STORAGE_FLAG_FINAL,
                      hk128, sizeof(hk128));

      /* Pack and push HK2 (Mag & Gyro) record into unified ring buffer */
      uint8_t hk2_128[128];
      memset(hk2_128, 0, sizeof(hk2_128));
      memcpy(&hk2_128[0], &data->pkt.adc2.gyro_x, 12);
      hk2_128[70] = 0x66;
      hk2_128[71] = 0xBB;
      memcpy(&hk2_128[72], &g_hk_flash_seq, 4);
      memcpy(&hk2_128[76], &now_ms, 4);

      uint16_t crc2 = hk_flash_crc16(hk2_128, 124);
      hk2_128[124] = (uint8_t)(crc2 & 0xFF);
      hk2_128[125] = (uint8_t)(crc2 >> 8);
      hk2_128[126] = 0xAA;
      hk2_128[127] = 0xCC;

      storage_rb_push(STORAGE_PART_HK2, STORAGE_FLAG_APPEND | STORAGE_FLAG_FINAL,
                      hk2_128, sizeof(hk2_128));
      return 0;
    }

  return -1;
}

int telemetry_rb_read(struct telemetry_envelope_s *data)
{
  (void)data;
  return -1; /* Drained directly by unified storage consumer */
}

int telemetry_rb_count(void)
{
  return storage_rb_count();
}

int ensure_hk_partition_mounted(void)
{
  /* Partitions are mounted on demand (1 at a time) to preserve SRAM */
  return 0;
}

void hk_rb_drain_to_flash(void)
{
  storage_rb_flush();
}
