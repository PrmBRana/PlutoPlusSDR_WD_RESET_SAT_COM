#include "ring_buffer.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <time.h>
#include <nuttx/mutex.h>

/*
 * ring_buffer.c
 *
 * Logic mirrors the Verilog CircularBuffer exactly:
 *
 *   Verilog                     C equivalent
 *   ──────────────────────────  ─────────────────────────────────
 *   wr_en && !full  → write     rb_write()  checks count < RB_DEPTH
 *   rd_en && !empty → read      rb_read()   checks count > 0
 *   wr_ptr <= wr_ptr + 1        g_rb.wr_ptr++ then % RB_DEPTH for slot
 *   rd_ptr <= rd_ptr + 1        g_rb.rd_ptr++ then % RB_DEPTH for slot
 *   count +1 / -1 / hold        count updated after every write/read
 *   full  = (count == DEPTH)    rb_full()
 *   empty = (count == 0)        rb_empty()
 */

static struct ring_buffer_s g_rb;
static mutex_t              g_rb_lock;
static bool                 g_rb_initialized = false;

/* ------------------------------------------------------------------ */
void rb_init(void)
{
  if (!g_rb_initialized)
    {
      memset(&g_rb, 0, sizeof(g_rb));
      g_rb.wr_ptr = 0;
      g_rb.rd_ptr = 0;
      g_rb.count  = 0;
      nxmutex_init(&g_rb_lock);
      g_rb_initialized = true;
    }
}

/* ------------------------------------------------------------------ */
/*  Write one chunk                                                    */
/*  Verilog: if (wr_en && !full) { mem[wr_ptr] <= wr_data;           */
/*                                  wr_ptr <= wr_ptr + 1; count++ }  */
/* ------------------------------------------------------------------ */
int rb_write(const struct image_chunk_s *c)
{
  if (!c) return -1;

  nxmutex_lock(&g_rb_lock);

  /* full check — mirrors: assign full = (count == DEPTH) */
  if (g_rb.count >= RB_DEPTH)
    {
      nxmutex_unlock(&g_rb_lock);
      return -1;   /* full — caller retries */
    }

  uint32_t slot = g_rb.wr_ptr % RB_DEPTH;           /* slot index      */
  memcpy(&g_rb.slots[slot], c, sizeof(*c));
  g_rb.wr_ptr++;                                     /* advance pointer */
  g_rb.count++;                                      /* update count    */

  nxmutex_unlock(&g_rb_lock);
  return 0;
}

/* ------------------------------------------------------------------ */
/*  Read one chunk                                                     */
/*  Verilog: if (rd_en && !empty) { rd_data = mem[rd_ptr];           */
/*                                   rd_ptr <= rd_ptr + 1; count-- } */
/* ------------------------------------------------------------------ */
int rb_read(struct image_chunk_s *c)
{
  if (!c) return -1;

  nxmutex_lock(&g_rb_lock);

  /* empty check — mirrors: assign empty = (count == 0) */
  if (g_rb.count == 0)
    {
      nxmutex_unlock(&g_rb_lock);
      return -1;   /* empty — caller retries */
    }

  uint32_t slot = g_rb.rd_ptr % RB_DEPTH;           /* slot index      */
  memcpy(c, &g_rb.slots[slot], sizeof(*c));
  g_rb.rd_ptr++;                                     /* advance pointer */
  g_rb.count--;                                      /* update count    */

  nxmutex_unlock(&g_rb_lock);
  return 0;
}

/* ------------------------------------------------------------------ */
int rb_count(void)
{
  nxmutex_lock(&g_rb_lock);
  int n = (int)g_rb.count;
  nxmutex_unlock(&g_rb_lock);
  return n;
}

int rb_full(void)
{
  nxmutex_lock(&g_rb_lock);
  int f = (g_rb.count >= RB_DEPTH) ? 1 : 0;
  nxmutex_unlock(&g_rb_lock);
  return f;
}

int rb_empty(void)
{
  nxmutex_lock(&g_rb_lock);
  int e = (g_rb.count == 0) ? 1 : 0;
  nxmutex_unlock(&g_rb_lock);
  return e;
}

/* ========================================================================= */
/* Telemetry Ring Buffer Implementation (Shared Inter-App)                   */
/* ========================================================================= */

#include <telemetry_rb.h>

struct telemetry_rb_s g_telemetry_rb;
sem_t                 g_telemetry_sem;
static mutex_t        g_telem_lock;
static bool           g_telem_initialized = false;

void telemetry_rb_init(void)
{
  if (!g_telem_initialized)
    {
      nxmutex_init(&g_telem_lock);
      memset(&g_telemetry_rb, 0, sizeof(g_telemetry_rb));
      sem_init(&g_telemetry_sem, 0, 0);
      g_telem_initialized = true;
    }
}

int telemetry_rb_write(const struct telemetry_envelope_s *data)
{
  if (!data)
    {
      return -1;
    }

  nxmutex_lock(&g_telem_lock);

  if (g_telemetry_rb.count >= TELEM_RB_DEPTH)
    {
      /* Buffer full: drop oldest packet */
      g_telemetry_rb.rd_ptr = (g_telemetry_rb.rd_ptr + 1) % TELEM_RB_DEPTH;
      g_telemetry_rb.count--;
    }

  uint32_t slot = g_telemetry_rb.wr_ptr % TELEM_RB_DEPTH;
  g_telemetry_rb.slots[slot] = *data;
  g_telemetry_rb.wr_ptr++;
  g_telemetry_rb.count++;

  nxmutex_unlock(&g_telem_lock);
  return 0;
}

int telemetry_rb_read(struct telemetry_envelope_s *data)
{
  if (!data)
    {
      return -1;
    }

  nxmutex_lock(&g_telem_lock);

  if (g_telemetry_rb.count == 0)
    {
      nxmutex_unlock(&g_telem_lock);
      return -1; /* Empty */
    }

  uint32_t slot = g_telemetry_rb.rd_ptr % TELEM_RB_DEPTH;
  *data = g_telemetry_rb.slots[slot];
  g_telemetry_rb.rd_ptr++;
  g_telemetry_rb.count--;

  nxmutex_unlock(&g_telem_lock);
  return 0;
}

int telemetry_rb_count(void)
{
  nxmutex_lock(&g_telem_lock);
  int c = (int)g_telemetry_rb.count;
  nxmutex_unlock(&g_telem_lock);
  return c;
}

/* ========================================================================= */
/* Camera Ring Buffer -> Flash CAM Partition (/mnt/camera) Consumer Helpers  */
/* ========================================================================= */

static mutex_t g_cam_flash_lock = NXMUTEX_INITIALIZER;
static int     g_cam_fd[3]      = { -1, -1, -1 };
static size_t  g_cam_written[3] = { 0, 0, 0 };
static bool    g_cam_mounted    = false;

int ensure_camera_partition_mounted(void)
{
  struct stat st;

  if (g_cam_mounted && stat("/mnt/camera", &st) == 0)
    {
      return 0;
    }

  if (stat("/mnt/camera", &st) != 0)
    {
      mkdir("/mnt/camera", 0777);
    }

  int ret = mount("/dev/camera", "/mnt/camera", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      int err = errno;
      if (err == EBUSY || err == EEXIST || err == ENOTDIR)
        {
          g_cam_mounted = true;
          return 0;
        }

      if (stat("/dev/camera", &st) == 0)
        {
          printf("[LITTLEFS CAM] WARN: autoformat /dev/camera failed (errno=%d), trying forceformat...\n", err);
          ret = mount("/dev/camera", "/mnt/camera", "littlefs", 0, "forceformat");
          if (ret == 0 || errno == EBUSY || errno == EEXIST || errno == ENOTDIR)
            {
              g_cam_mounted = true;
              return 0;
            }
          err = errno;
        }
      return -err;
    }

  g_cam_mounted = true;
  return 0;
}

void cam_rb_drain_to_flash(void)
{
  struct image_chunk_s chunk;

  if (rb_empty())
    {
      return;
    }

  nxmutex_lock(&g_cam_flash_lock);
  ensure_camera_partition_mounted();

  while (rb_read(&chunk) == 0)
    {
      uint32_t fid = (chunk.frame_id == 2) ? 2 : 1;
      const char *path = (fid == 2) ? "/mnt/camera/capture_02.raw"
                                    : "/mnt/camera/capture_01.raw";

      if (chunk.is_meta)
        {
          if (g_cam_fd[fid] >= 0)
            {
              fsync(g_cam_fd[fid]);
              close(g_cam_fd[fid]);
              g_cam_fd[fid] = -1;
            }
          g_cam_written[fid] = 0;
          g_cam_fd[fid] = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
          if (g_cam_fd[fid] < 0)
            {
              g_cam_mounted = false;
              umount("/mnt/camera");
              mount("/dev/camera", "/mnt/camera", "littlefs", 0, "forceformat");
              g_cam_mounted = true;
              g_cam_fd[fid] = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
            }
          continue;
        }

      if (g_cam_fd[fid] < 0)
        {
          g_cam_fd[fid] = open(path, O_WRONLY | O_CREAT | O_APPEND, 0666);
          if (g_cam_fd[fid] < 0)
            {
              g_cam_mounted = false;
              umount("/mnt/camera");
              mount("/dev/camera", "/mnt/camera", "littlefs", 0, "forceformat");
              g_cam_mounted = true;
              g_cam_fd[fid] = open(path, O_WRONLY | O_CREAT | O_APPEND, 0666);
            }
        }

      if (g_cam_fd[fid] >= 0 && chunk.len > 0)
        {
          ssize_t w = write(g_cam_fd[fid], chunk.data, chunk.len);
          if (w < 0)
            {
              close(g_cam_fd[fid]);
              g_cam_mounted = false;
              umount("/mnt/camera");
              mount("/dev/camera", "/mnt/camera", "littlefs", 0, "forceformat");
              g_cam_mounted = true;
              g_cam_fd[fid] = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
              if (g_cam_fd[fid] >= 0)
                {
                  w = write(g_cam_fd[fid], chunk.data, chunk.len);
                }
            }
          if (w > 0)
            {
              g_cam_written[fid] += (size_t)w;
            }
        }

      if (chunk.is_last)
        {
          if (g_cam_fd[fid] >= 0)
            {
              fsync(g_cam_fd[fid]);
              close(g_cam_fd[fid]);
              g_cam_fd[fid] = -1;
            }
          struct stat st;
          long disk_sz = (stat(path, &st) == 0) ? (long)st.st_size : -1L;
          printf("[LITTLEFS CAM] Stored Image %lu -> %s (%u B written, %ld B verified in Flash CAM partition)\n",
                 (unsigned long)fid, path, (unsigned)g_cam_written[fid], disk_sz);
        }
    }

  nxmutex_unlock(&g_cam_flash_lock);
}

void cam_rb_close_all_files(void)
{
  cam_rb_drain_to_flash();
  nxmutex_lock(&g_cam_flash_lock);
  for (int fid = 1; fid <= 2; fid++)
    {
      if (g_cam_fd[fid] >= 0)
        {
          fsync(g_cam_fd[fid]);
          close(g_cam_fd[fid]);
          g_cam_fd[fid] = -1;
        }
    }
  nxmutex_unlock(&g_cam_flash_lock);
}

/* ========================================================================= */
/* HK Telemetry Ring Buffer -> Flash HK Partition (/mnt/hk) Consumer Helpers */
/* ========================================================================= */

static mutex_t  g_hk_flash_lock = NXMUTEX_INITIALIZER;
static bool     g_hk_mounted    = false;
static uint32_t g_hk_flash_seq  = 0;

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

int ensure_hk_partition_mounted(void)
{
  struct stat st;

  if (g_hk_mounted && stat("/mnt/hk", &st) == 0)
    {
      return 0;
    }

  if (stat("/mnt/hk", &st) != 0)
    {
      mkdir("/mnt/hk", 0777);
    }

  int ret = mount("/dev/hk", "/mnt/hk", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      int err = errno;
      if (err == EBUSY || err == EEXIST || err == ENOTDIR)
        {
          g_hk_mounted = true;
          return 0;
        }

      if (stat("/dev/hk", &st) == 0)
        {
          printf("[LITTLEFS HK] WARN: autoformat /dev/hk failed (errno=%d), trying forceformat...\n", err);
          ret = mount("/dev/hk", "/mnt/hk", "littlefs", 0, "forceformat");
          if (ret == 0 || errno == EBUSY || errno == EEXIST || errno == ENOTDIR)
            {
              g_hk_mounted = true;
              return 0;
            }
          err = errno;
        }
      return -err;
    }

  g_hk_mounted = true;
  return 0;
}

void hk_rb_drain_to_flash(void)
{
  struct telemetry_envelope_s env;
  static struct adc1_packet_s s_cached_adc1;
  static bool s_have_adc1 = false;

  if (telemetry_rb_count() == 0)
    {
      return;
    }

  nxmutex_lock(&g_hk_flash_lock);
  ensure_hk_partition_mounted();

  while (telemetry_rb_read(&env) == 0)
    {
      if (env.footer == TELEM_FOOTER_ADC1)
        {
          s_cached_adc1 = env.pkt.adc1;
          s_have_adc1 = true;
        }
      else if (env.footer == TELEM_FOOTER_ADC2)
        {
          const char *filename = "/mnt/hk/telemetry.bin";
          uint8_t hk128[128];
          memset(hk128, 0, sizeof(hk128));

          if (s_have_adc1)
            {
              memcpy(&hk128[0], s_cached_adc1.data, 32);
            }
          hk128[32] = 0x55;
          hk128[33] = 0xAA;

          memcpy(&hk128[34], env.pkt.adc2.data, 24);
          memcpy(&hk128[58], &env.pkt.adc2.gyro_x, 12);

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

          int fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
          if (fd < 0)
            {
              g_hk_mounted = false;
              umount("/mnt/hk");
              mount("/dev/hk", "/mnt/hk", "littlefs", 0, "forceformat");
              g_hk_mounted = true;
              fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
            }

          if (fd >= 0)
            {
              ssize_t written = write(fd, hk128, sizeof(hk128));
              fsync(fd);
              close(fd);

              if (written < 0)
                {
                  g_hk_mounted = false;
                  umount("/mnt/hk");
                  mount("/dev/hk", "/mnt/hk", "littlefs", 0, "forceformat");
                  g_hk_mounted = true;
                  fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
                  if (fd >= 0)
                    {
                      written = write(fd, hk128, sizeof(hk128));
                      fsync(fd);
                      close(fd);
                    }
                }

              if (written == (ssize_t)sizeof(hk128))
                {
                  struct stat st;
                  long fsz = (stat(filename, &st) == 0) ? (long)st.st_size : -1L;
                  printf("[LITTLEFS HK] Stored 128B HK Record #%lu -> %s (File Size: %ld B, %ld Records)\n",
                         (unsigned long)g_hk_flash_seq, filename, fsz, (fsz > 0) ? (fsz / 128) : 0L);
                }
            }
        }
    }

  nxmutex_unlock(&g_hk_flash_lock);
}