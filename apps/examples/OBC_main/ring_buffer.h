/****************************************************************************
 * apps/examples/OBC_main/ring_buffer.h
 *
 * Unified Single Ring Buffer + Semaphore Storage Manager for all 11 MT25Q
 * external flash partitions (HK1, HK2, ADCS, CAM_RGB, CAM_NIR, EPDM, DPM,
 * RSV_Table, Imp_Flags, Part_Size, Seek_Ptr).
 *
 * Architecture:
 *   - ONE bounded ring buffer in RAM (STORAGE_RB_DEPTH x STORAGE_CHUNK_SIZE)
 *     shared by all producers (HK ADC/IMU, Camera, EPDM, ADCS, DPM, etc.).
 *   - Each slot carries a `part` destination tag + `flags` (APPEND / TRUNC /
 *     FINAL).
 *   - Two semaphores coordinate producer <-> single storage consumer:
 *       * g_storage_items_sem : posted on push, wakes consumer (0% CPU idle)
 *       * g_storage_space_sem : backpressure when all slots are full
 *   - Only ONE LittleFS partition is ever mounted at a time inside the
 *     storage consumer, capping LittleFS RAM at ~1.5-2 KB total across all
 *     11 partitions and eliminating ENOMEM on 32 KB SRAM1.
 ****************************************************************************/

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <telemetry_rb.h>

/* ========================================================================= */
/* Unified Storage Ring Buffer Definitions                                   */
/* ========================================================================= */

#define STORAGE_RB_DEPTH    4     /* 4 slots x 256 B = 1 KB RAM (power of 2) */
#define STORAGE_CHUNK_SIZE  256   /* Max payload bytes per ring buffer slot  */

/* Destination flash partition tag carried in every ring buffer slot */

typedef enum
{
  STORAGE_PART_HK1       = 0,  /* /dev/hk1       -> /mnt/hk1/telemetry.bin   */
  STORAGE_PART_HK2       = 1,  /* /dev/hk2       -> /mnt/hk2/imu.bin         */
  STORAGE_PART_ADCS      = 2,  /* /dev/adcs      -> /mnt/adcs/adcs.bin       */
  STORAGE_PART_CAM_RGB   = 3,  /* /dev/cam_rgb   -> /mnt/cam_rgb/capture_01.raw */
  STORAGE_PART_CAM_NIR   = 4,  /* /dev/cam_nir   -> /mnt/cam_nir/capture_02.raw */
  STORAGE_PART_EPDM      = 5,  /* /dev/epdm      -> /mnt/epdm/epdm.bin       */
  STORAGE_PART_DPM       = 6,  /* /dev/dpm       -> /mnt/dpm/dpm.bin         */
  STORAGE_PART_RSV_TABLE = 7,  /* /dev/rsv_table -> /mnt/rsv_table/rsv.bin   */
  STORAGE_PART_IMP_FLAGS = 8,  /* /dev/imp_flags -> /mnt/imp_flags/flags.bin */
  STORAGE_PART_PART_SIZE = 9,  /* /dev/part_size -> /mnt/part_size/size.bin  */
  STORAGE_PART_SEEK_PTR  = 10, /* /dev/seek_ptr  -> /mnt/seek_ptr/seek.bin   */
  STORAGE_PART_COUNT     = 11,
  STORAGE_PART_NONE      = 0xFF
} storage_part_t;

/* Per-slot action flags (bitmask) */

#define STORAGE_FLAG_APPEND  0x00U  /* Append data to destination file       */
#define STORAGE_FLAG_TRUNC   0x01U  /* Truncate/recreate file before writing */
#define STORAGE_FLAG_FINAL   0x02U  /* fsync + close + unmount after writing */

/* Unified ring buffer slot */

struct storage_item_s
{
  uint8_t  part;                     /* storage_part_t destination tag       */
  uint8_t  flags;                    /* STORAGE_FLAG_* bitmask               */
  uint16_t len;                      /* Valid bytes in data[] (0..256)       */
  uint8_t  data[STORAGE_CHUNK_SIZE]; /* Payload bytes                        */
};

/* Unified Storage API */

void storage_rb_init(void);
int  storage_rb_push(storage_part_t part, uint8_t flags,
                     const void *data, uint16_t len);
int  storage_rb_count(void);
void storage_rb_flush(void);         /* Drain queue & unmount active part    */
int  storage_consumer_run(void);     /* Dedicated storage daemon main loop   */

/* ========================================================================= */
/* Legacy Camera / Telemetry Compatibility Definitions                       */
/* ========================================================================= */

#define RB_DEPTH    STORAGE_RB_DEPTH
#define CHUNK_SIZE  STORAGE_CHUNK_SIZE

struct image_chunk_s
{
  uint8_t  data[CHUNK_SIZE];
  uint16_t len;
  uint32_t frame_id;
  uint8_t  is_last;
  uint8_t  is_meta;
};

void rb_init(void);
int  rb_write(const struct image_chunk_s *c);
int  rb_read(struct image_chunk_s *c);
int  rb_count(void);
int  rb_full(void);
int  rb_empty(void);

int  ensure_camera_partition_mounted(void);
void unmount_camera_partitions(void);
void cam_rb_drain_to_flash(void);
void cam_rb_close_all_files(void);

int  ensure_hk_partition_mounted(void);
void hk_rb_drain_to_flash(void);

#endif /* RING_BUFFER_H */
