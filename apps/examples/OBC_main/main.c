/****************************************************************************
 * apps/examples/OBC_main/main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 ****************************************************************************/

/****************************************************************************
 * Overview
 *
 *   OBC_main is the On-Board Computer's telemetry producer task.
 *
 *   The M4 CPU uses TWO execution paths:
 *
 *     1. Main OBC thread
 *        - Continuously processes M0+ radio logs
 *        - Processes ground commands
 *        - Handles other OBC tasks
 *
 *     2. Telemetry pthread
 *        - Sleeps for 90 seconds
 *        - Wakes up
 *        - Reads ADC1
 *        - Reads ADC2
 *        - Reads gyroscope
 *        - Reads magnetometer
 *        - Builds Beacon 1 and Beacon 2
 *        - Stores telemetry in local ring buffer
 *        - Updates live telemetry
 *        - Notifies M0+ through IPCC
 *        - Goes back to sleep
 *
 *   This means telemetry acquisition does not block the main OBC loop.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <semaphore.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <dirent.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/iam20380.h>
#include <nuttx/sensors/mmc5983ma.h>

#include <telemetry_rb.h>
#include "ring_buffer.h"
#include "Sring_buffer.h"
#include <termios.h>
#include <poll.h>
#include <time.h>

#define CAM_CHUNK_SIZE              128U
#define CAM_NUM_CHUNKS              69U

#include <pthread.h>
#include <sched.h>

#include <arch/board/board.h>

/****************************************************************************
 * Board-specific SPI bus initialization
 ****************************************************************************/

FAR struct spi_dev_s *stm32wl5_spibus_initialize(int bus);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC device paths and geometry */

#define ADC1_DEVPATH        "/dev/adc0"
#define ADC2_DEVPATH        "/dev/adc1"
#define ADS7953_CHANNELS    16

/* ADS7953 full-scale */

#define ADC_VREF_FULLSCALE  5.000f
#define ADC_MAX_COUNTS      4095.0f

/* NTC parameters */

#define NTC_R_PULLUP        10000.0f
#define NTC_V_SUPPLY        3.300f
#define NTC_R0              10000.0f
#define NTC_BETA            3950.0f
#define NTC_T0              298.15f

/* Detection thresholds */

#define SOLAR_V_THRESHOLD   0.250f
#define SOLAR_I_THRESHOLD   0.015f
#define IDLE_I_THRESHOLD    0.010f

/* Telemetry acquisition cadence */

#define TELEMETRY_INTERVAL_SEC  90

/* SPI2 chip-select IDs */

#define GYRO_CS_DEVID       SPIDEV_USER(3)
#define MAG_CS_DEVID        SPIDEV_USER(2)

/* SPI clocks */

#define GYRO_SPI_FREQ       8000000
#define MAG_SPI_FREQ        10000000

/* Telemetry packing scales */

#define MAG_PACKET_SCALE    10.0f
#define GYRO_PACKET_SCALE   100.0f

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

enum adc_channel_type_e
{
  CHANNEL_TYPE_VOLT = 0,
  CHANNEL_TYPE_SOLAR_V,
  CHANNEL_TYPE_TEMP,
  CHANNEL_TYPE_CURR,
  CHANNEL_TYPE_SOLAR_I,
  CHANNEL_TYPE_NC
};

enum sensor_status_e
{
  STATUS_ACTIVE_OK = 0,
  STATUS_DISCONNECTED,
  STATUS_NO_DATA_DARK,
  STATUS_IDLE_NO_LOAD,
  STATUS_NC_UNUSED,
  STATUS_FAULT
};

struct adc_channel_cfg_s
{
  const char              *name;
  enum adc_channel_type_e  type;
  const char              *unit;
  float                    slope;
  float                    offset;
  bool                     is_nc;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* -------------------------------------------------------------------------
 * Telemetry thread state
 * -------------------------------------------------------------------------
 */

static pthread_t g_telemetry_thread __attribute__((unused));

/* Persistent telemetry scan counter.
 *
 * IMPORTANT:
 * This must NOT be local to perform_telemetry_cycle(), otherwise it
 * resets to zero every time the function is called.
 */

static int g_scan_count = 0;

/* -------------------------------------------------------------------------
 * Shared SPI / IMU state
 * -------------------------------------------------------------------------
 *
 * These used to be local variables inside obc_main_main().
 * They must be global/static because telemetry_thread() accesses them.
 */

static FAR struct spi_dev_s *g_spi2 = NULL;

static struct iam20380_config_s g_gyro_cfg;
static struct mmc5983ma_config_s g_mag_cfg;

static bool g_gyro_initialized = false;
static bool g_mag_initialized = false;

/* -------------------------------------------------------------------------
 * Beacon/IPCC protection
 * -------------------------------------------------------------------------
 *
 * Both the telemetry thread and process_ground_commands() can request
 * an IPCC beacon. Protect the live telemetry update + IPCC notification
 * as one operation.
 */

static pthread_mutex_t g_beacon_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile bool   g_camera_capture_active = false;

/****************************************************************************
 * ADC1 channel map
 *
 * CH00-CH07 : Voltage monitors
 * CH08-CH15 : NTC temperature sensors
 ****************************************************************************/

static const struct adc_channel_cfg_s
g_adc1_cfg[ADS7953_CHANNELS] =
{
  { "ADC_BAT_MON",    CHANNEL_TYPE_VOLT,    "V",  1.0f, 0.0f, false },
  { "TOTAL_SOLAR_V",  CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "RAW_VOLT",       CHANNEL_TYPE_VOLT,    "V",  1.0f, 0.0f, false },
  { "SP5_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP4_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP3_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP1_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "SP2_VOLT",       CHANNEL_TYPE_SOLAR_V, "V",  1.0f, 0.0f, false },
  { "ANT_TEMP",       CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "BATT_TEMP",      CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP_BPB",       CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP1",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP5",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP4",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP3",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
  { "TEMP2",          CHANNEL_TYPE_TEMP,    "C",  1.0f, 0.0f, false },
};

/****************************************************************************
 * ADC2 channel map
 ****************************************************************************/

static const struct adc_channel_cfg_s
g_adc2_cfg[ADS7953_CHANNELS] =
{
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "UNREG_I",        CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SP4_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "MAIN_3V3_I",     CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "MISSION_3V3_I",  CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SPT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "5V_I",           CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SP2_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "SP1_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "RAW_I",          CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "NC",             CHANNEL_TYPE_NC,      "--", 0.0f, 0.0f, true  },
  { "SP5_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
  { "BAT_I",          CHANNEL_TYPE_CURR,    "A",  1.0f, 0.0f, false },
  { "SP3_I",          CHANNEL_TYPE_SOLAR_I, "A",  1.0f, 0.0f, false },
};

/****************************************************************************
 * Forward Declarations
 ****************************************************************************/

static int16_t pack_i16(float value);

static int read_adc_chip(const char *devpath,
                         uint16_t channels[ADS7953_CHANNELS]);

static float convert_channel(uint16_t raw_counts,
                             const struct adc_channel_cfg_s *cfg,
                             float *v_pin_out,
                             enum sensor_status_e *status_out);

static void display_adc1_telemetry(
                             uint16_t raw_data[ADS7953_CHANNELS]);

static void display_adc2_telemetry(
                             uint16_t raw_data[ADS7953_CHANNELS],
                             const struct iam20380_data_s *gyro,
                             const struct mmc5983ma_data_s *mag,
                             bool gyro_ok,
                             bool mag_ok);

static bool init_gyro(FAR struct spi_dev_s *spi,
                      FAR const struct iam20380_config_s *cfg);

static bool init_mag(FAR struct spi_dev_s *spi,
                     FAR const struct mmc5983ma_config_s *cfg);

static bool gyro_probe_cs(FAR struct spi_dev_s *spi,
                          FAR struct iam20380_config_s *cfg);

static int send_telemetry_to_ringbuffer(
                         uint16_t adc0_raw[ADS7953_CHANNELS],
                         uint16_t adc1_raw[ADS7953_CHANNELS],
                         const struct iam20380_data_s *gyro,
                         const struct mmc5983ma_data_s *mag,
                         bool gyro_ok,
                         bool mag_ok,
                         int scan_count);

static bool check_camera_data_available(void);

static void process_ground_commands(void);

static void perform_telemetry_cycle(void);

static void *telemetry_thread(void *arg) __attribute__((unused));

/****************************************************************************
 * Name: telemetry_thread
 *
 * Description:
 *   Dedicated periodic telemetry acquisition thread.
 *
 *   The thread sleeps for 90 seconds and then performs exactly one
 *   telemetry acquisition cycle.
 *
 *   The main OBC thread is therefore free to handle radio logs and
 *   ground commands while this thread sleeps.
 ****************************************************************************/

static void *telemetry_thread(void *arg)
{
  (void)arg;

  printf("\n");
  printf("[TELEMETRY] ================================================\n");
  printf("[TELEMETRY] Telemetry thread started\n");
  printf("[TELEMETRY] Acquisition interval = %d seconds\n",
         TELEMETRY_INTERVAL_SEC);
  printf("[TELEMETRY] ================================================\n");

  while (1)
    {
      /* Pause telemetry acquisition while M0+ is actively downlinking or camera is capturing */
      while (sat_is_downlink_active() || g_camera_capture_active)
        {
          usleep(100000); /* 100 ms yield */
        }

      /* 1. Perform telemetry acquisition and update live buffer FIRST */
      perform_telemetry_cycle();

      printf("[TELEMETRY] Acquisition cycle completed\n");

      /* 2. Sleep for interval until next periodic acquisition */
      printf("[TELEMETRY] Sleeping for %d seconds...\n",
             TELEMETRY_INTERVAL_SEC);

      sleep(TELEMETRY_INTERVAL_SEC);

      printf("\n");
      printf("[TELEMETRY] ================================================\n");
      printf("[TELEMETRY] %d seconds elapsed - waking up\n",
             TELEMETRY_INTERVAL_SEC);
      printf("[TELEMETRY] ================================================\n");
    }

  return NULL;
}

/****************************************************************************
 * Name: perform_telemetry_cycle
 *
 * Description:
 *   Perform one complete telemetry acquisition cycle.
 *
 *   This function is called ONLY by telemetry_thread().
 *
 *   Hardware initialization is NOT performed here except retrying a sensor
 *   if it previously failed.
 ****************************************************************************/

static void perform_telemetry_cycle(void)
{
  uint16_t adc1_data[ADS7953_CHANNELS];
  uint16_t adc2_data[ADS7953_CHANNELS];

  struct iam20380_data_s gyro_data;
  struct mmc5983ma_data_s mag_data;

  bool adc1_read_ok = false;
  bool adc2_read_ok = false;
  bool gyro_read_ok = false;
  bool mag_read_ok = false;

  int ret;

  memset(adc1_data, 0, sizeof(adc1_data));
  memset(adc2_data, 0, sizeof(adc2_data));
  memset(&gyro_data, 0, sizeof(gyro_data));
  memset(&mag_data, 0, sizeof(mag_data));

  /*
   * Persistent scan counter.
   */

  g_scan_count++;

  printf("\n");
  printf("==============================================================================================\n");
  printf("  [OBC_MAIN] TELEMETRY ACQUISITION SCAN #%d\n",
         g_scan_count);
  printf("==============================================================================================\n");

  /**************************************************************************
   * 1. ADC1
   **************************************************************************/

  ret = read_adc_chip(ADC1_DEVPATH, adc1_data);

  if (ret >= 0)
    {
      adc1_read_ok = true;

      printf("[OBC_MAIN] ADC1 read successful: %d samples\n", ret);

      display_adc1_telemetry(adc1_data);
    }
  else
    {
      printf("[OBC_MAIN] ERROR: Failed to read ADC1 (%s), ret=%d\n",
             ADC1_DEVPATH, ret);
    }

  /**************************************************************************
   * 2. ADC2
   **************************************************************************/

  ret = read_adc_chip(ADC2_DEVPATH, adc2_data);

  if (ret >= 0)
    {
      adc2_read_ok = true;

      printf("[OBC_MAIN] ADC2 read successful: %d samples\n", ret);
    }
  else
    {
      printf("[OBC_MAIN] ERROR: Failed to read ADC2 (%s), ret=%d\n",
             ADC2_DEVPATH, ret);
    }

  /**************************************************************************
   * 3. Retry IMU initialization if required
   *
   * Initialization normally happens once during startup.
   *
   * If a sensor failed initialization, we retry here on later telemetry
   * cycles instead of repeatedly initializing it every cycle.
   **************************************************************************/

  if (g_spi2 != NULL)
    {
      if (!g_gyro_initialized)
        {
          printf("[OBC_MAIN] Retrying gyro initialization...\n");

          g_gyro_initialized =
            init_gyro(g_spi2, &g_gyro_cfg);

          if (g_gyro_initialized)
            {
              printf("[OBC_MAIN] Gyro retry initialization successful\n");
            }
        }

      if (!g_mag_initialized)
        {
          printf("[OBC_MAIN] Retrying magnetometer initialization...\n");

          g_mag_initialized =
            init_mag(g_spi2, &g_mag_cfg);

          if (g_mag_initialized)
            {
              printf("[OBC_MAIN] Magnetometer retry initialization successful\n");
            }
        }
    }

  /**************************************************************************
   * 4. Read gyroscope
   **************************************************************************/

  if (g_spi2 != NULL && g_gyro_initialized)
    {
      ret = iam20380_read(g_spi2, &gyro_data);

      if (ret == OK)
        {
          gyro_read_ok = true;
        }
      else
        {
          printf("[OBC_MAIN] ERROR: iam20380_read failed (ret=%d), "
                 "will re-init on next telemetry cycle\n",
                 ret);

          g_gyro_initialized = false;
        }
    }

  /**************************************************************************
   * 5. Read magnetometer
   **************************************************************************/

  if (g_spi2 != NULL && g_mag_initialized)
    {
      ret = mmc5983ma_read(g_spi2, &mag_data);

      if (ret == OK)
        {
          mag_read_ok = true;
        }
      else
        {
          printf("[OBC_MAIN] ERROR: mmc5983ma_read failed (ret=%d), "
                 "will re-init on next telemetry cycle\n",
                 ret);

          g_mag_initialized = false;
        }
    }

  /**************************************************************************
   * 6. Display ADC2 + IMU
   **************************************************************************/

  if (adc2_read_ok)
    {
      display_adc2_telemetry(adc2_data,
                             &gyro_data,
                             &mag_data,
                             gyro_read_ok,
                             mag_read_ok);
    }

  /**************************************************************************
   * 7. Build/store/transmit telemetry
   *
   * adc1_read_ok is currently retained for diagnostics.  The existing
   * read_adc_chip() function clears the buffer on failure, so the existing
   * packet-building behavior is preserved.
   **************************************************************************/

  (void)adc1_read_ok;

  ret = send_telemetry_to_ringbuffer(adc1_data,
                                     adc2_data,
                                     &gyro_data,
                                     &mag_data,
                                     gyro_read_ok,
                                     mag_read_ok,
                                     g_scan_count);

  if (ret < 0)
    {
      printf("[OBC_MAIN] ERROR: send_telemetry_to_ringbuffer failed: %d\n",
             ret);
    }

  printf("\n");
  printf("[OBC_MAIN] [PRODUCER] Buffered Telemetry to Ring Buffer "
         "(Occupancy: %d/16)\n",
         telemetry_rb_count());

  /* Drain telemetry ring buffer to Flash HK partition (/mnt/hk/telemetry.bin) */
  hk_rb_drain_to_flash();

  printf("[OBC_MAIN] Telemetry scan #%d finished\n",
         g_scan_count);

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: pack_i16
 ****************************************************************************/

static int16_t pack_i16(float value)
{
  float r = roundf(value);

  if (r != r)
    {
      return 0;
    }

  if (r > 32767.0f)
    {
      return INT16_MAX;
    }

  if (r < -32768.0f)
    {
      return INT16_MIN;
    }

  return (int16_t)r;
}

/****************************************************************************
 * Name: read_adc_chip
 ****************************************************************************/

static int read_adc_chip(const char *devpath,
                         uint16_t channels[ADS7953_CHANNELS])
{
  struct adc_msg_s samples[ADS7953_CHANNELS];
  ssize_t nbytes;
  int nsamples;
  int fd;
  int ret;
  int i;

  memset(channels, 0,
         sizeof(uint16_t) * ADS7953_CHANNELS);

  fd = open(devpath, O_RDONLY);
  if (fd < 0)
    {
      int err = errno;

      printf("ERROR: Failed to open %s (errno: %d)\n",
             devpath, err);

      return -err;
    }

  /* Clear any stale or leftover samples in the driver FIFO so we only read fresh data */
  ioctl(fd, ANIOC_RESET_FIFO, 0);

  ret = ioctl(fd, ANIOC_TRIGGER, 0);

  if (ret < 0)
    {
      int err = errno;

      printf("ERROR: ANIOC_TRIGGER failed on %s (errno: %d)\n",
             devpath, err);

      close(fd);

      return -err;
    }

  nbytes = read(fd,
                samples,
                sizeof(samples));

  if (nbytes < 0)
    {
      int err = errno;

      printf("ERROR: Read failed on %s (errno: %d)\n",
             devpath, err);

      close(fd);

      return -err;
    }

  nsamples = nbytes / sizeof(struct adc_msg_s);

  for (i = 0; i < nsamples; i++)
    {
      uint8_t ch = samples[i].am_channel;

      if (ch < ADS7953_CHANNELS)
        {
          channels[ch] =
            (uint16_t)(samples[i].am_data & 0x0FFF);
        }
    }

  close(fd);

  return nsamples;
}

/****************************************************************************
 * Name: convert_channel
 ****************************************************************************/

static float convert_channel(uint16_t raw_counts,
                             const struct adc_channel_cfg_s *cfg,
                             float *v_pin_out,
                             enum sensor_status_e *status_out)
{
  float v_pin;

  if (cfg->is_nc)
    {
      if (v_pin_out)
        {
          *v_pin_out = 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_NC_UNUSED;
        }

      return 0.0f;
    }

  v_pin =
    ((float)raw_counts / ADC_MAX_COUNTS) *
    ADC_VREF_FULLSCALE;

  if (v_pin_out)
    {
      *v_pin_out = v_pin;
    }

  /* NTC temperature */

  if (cfg->type == CHANNEL_TYPE_TEMP)
    {
      if (raw_counts >= 4000 ||
          v_pin >= (NTC_V_SUPPLY - 0.05f))
        {
          if (status_out)
            {
              *status_out = STATUS_DISCONNECTED;
            }

          return 0.0f;
        }

      {
        float r_ntc;
        float steinhart;
        float temp_c;

        r_ntc =
          NTC_R_PULLUP *
          (v_pin / (NTC_V_SUPPLY - v_pin));

        if (r_ntc <= 0.0f)
          {
            if (status_out)
              {
                *status_out = STATUS_FAULT;
              }

            return -273.15f;
          }

        steinhart =
          (1.0f / NTC_T0) +
          (1.0f / NTC_BETA) *
          logf(r_ntc / NTC_R0);

        temp_c =
          (1.0f / steinhart) - 273.15f;

        if (status_out)
          {
            *status_out = STATUS_ACTIVE_OK;
          }

        return temp_c;
      }
    }

  /* Solar voltage */

  if (cfg->type == CHANNEL_TYPE_SOLAR_V)
    {
      if (v_pin < SOLAR_V_THRESHOLD)
        {
          if (status_out)
            {
              *status_out = STATUS_NO_DATA_DARK;
            }

          return 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_ACTIVE_OK;
        }

      return (v_pin * cfg->slope) +
             cfg->offset;
    }

  /* Solar current */

  if (cfg->type == CHANNEL_TYPE_SOLAR_I)
    {
      float current =
        (v_pin * cfg->slope) +
        cfg->offset;

      if (v_pin < SOLAR_I_THRESHOLD ||
          current < 0.010f)
        {
          if (status_out)
            {
              *status_out = STATUS_NO_DATA_DARK;
            }

          return 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_ACTIVE_OK;
        }

      return current;
    }

  /* General current */

  if (cfg->type == CHANNEL_TYPE_CURR)
    {
      float current =
        (v_pin * cfg->slope) +
        cfg->offset;

      if (v_pin < IDLE_I_THRESHOLD ||
          current < 0.005f)
        {
          if (status_out)
            {
              *status_out = STATUS_IDLE_NO_LOAD;
            }

          return 0.0f;
        }

      if (status_out)
        {
          *status_out = STATUS_ACTIVE_OK;
        }

      return current;
    }

  /* General voltage */

  if (status_out)
    {
      *status_out = STATUS_ACTIVE_OK;
    }

  return (v_pin * cfg->slope) +
         cfg->offset;
}

/****************************************************************************
 * Name: send_telemetry_to_ringbuffer
 ****************************************************************************/

static int send_telemetry_to_ringbuffer(
                         uint16_t adc0_raw[ADS7953_CHANNELS],
                         uint16_t adc1_raw[ADS7953_CHANNELS],
                         const struct iam20380_data_s *gyro,
                         const struct mmc5983ma_data_s *mag,
                         bool gyro_ok,
                         bool mag_ok,
                         int scan_count)
{
  static const uint8_t adc2_active_map[ADC2_ACTIVE_CHANNELS] =
  {
    1,   /* [0] UNREG_I */
    3,   /* [1] MAIN_3V3_I */
    8,   /* [2] 5V_I */
    14,  /* [3] BAT_I */
    10,  /* [4] SP1_I */
    9,   /* [5] SP2_I */
    15,  /* [6] SP3_I */
    2,   /* [7] SP4_I */
    13,  /* [8] SP5_I */
    11,  /* [9] RAW_I / BusI */
    5,   /* [10] MISSION_3V3_I (Flag 1) */
    6    /* [11] SPT_I         (Flag 2) */
  };

  struct telemetry_envelope_s env_adc1;
  struct telemetry_envelope_s env_adc2;

  float v_pin;
  enum sensor_status_e status;

  int ch;
  int i;

  /**************************************************************************
   * 1. Beacon 1
   *
   * 16 channels x 2 bytes + 2-byte footer = 34 bytes
   **************************************************************************/

  memset(&env_adc1, 0, sizeof(env_adc1));

  env_adc1.len =
    sizeof(struct adc1_packet_s);

  env_adc1.footer =
    TELEM_FOOTER_ADC1;

  for (ch = 0;
       ch < ADC1_CHANNELS_COUNT;
       ch++)
    {
      float val;

      val =
        convert_channel(adc0_raw[ch],
                        &g_adc1_cfg[ch],
                        &v_pin,
                        &status);

      env_adc1.pkt.adc1.data[ch] =
        pack_i16(val * 100.0f);
    }

  env_adc1.pkt.adc1.footer =
    TELEM_FOOTER_ADC1;

  /*
   * Local telemetry ring buffer.
   *
   * This always stores Beacon 1 for flash.
   */

  telemetry_rb_write(&env_adc1);

  sem_post(&g_telemetry_sem);

  /**************************************************************************
   * 2. Beacon 2
   *
   * 12 currents = 24 bytes
   * 3 gyro       =  6 bytes
   * 3 magneto    =  6 bytes
   * footer       =  2 bytes
   *
   * Total = 38 bytes
   **************************************************************************/

  memset(&env_adc2, 0, sizeof(env_adc2));

  env_adc2.len =
    sizeof(struct adc2_packet_s);

  env_adc2.footer =
    TELEM_FOOTER_ADC2;

  /* 12 active current channels */

  for (i = 0;
       i < ADC2_ACTIVE_CHANNELS;
       i++)
    {
      float val;

      ch = adc2_active_map[i];

      val =
        convert_channel(adc1_raw[ch],
                        &g_adc2_cfg[ch],
                        &v_pin,
                        &status);

      env_adc2.pkt.adc2.data[i] =
        pack_i16(val * 100.0f);
    }

  /* Gyroscope */

  if (gyro_ok && gyro != NULL)
    {
      float dps_x;
      float dps_y;
      float dps_z;

      dps_x =
        iam20380_counts_to_dps(gyro->x);

      dps_y =
        iam20380_counts_to_dps(gyro->y);

      dps_z =
        iam20380_counts_to_dps(gyro->z);

      env_adc2.pkt.adc2.gyro_x =
        pack_i16(dps_x * GYRO_PACKET_SCALE);

      env_adc2.pkt.adc2.gyro_y =
        pack_i16(dps_y * GYRO_PACKET_SCALE);

      env_adc2.pkt.adc2.gyro_z =
        pack_i16(dps_z * GYRO_PACKET_SCALE);
    }
  else
    {
      env_adc2.pkt.adc2.gyro_x = 0;
      env_adc2.pkt.adc2.gyro_y = 0;
      env_adc2.pkt.adc2.gyro_z = 0;
    }

  /* Magnetometer */

  if (mag_ok && mag != NULL)
    {
      float ut_x;
      float ut_y;
      float ut_z;

      ut_x =
        mmc5983ma_counts_to_ut(mag->x);

      ut_y =
        mmc5983ma_counts_to_ut(mag->y);

      ut_z =
        mmc5983ma_counts_to_ut(mag->z);

      env_adc2.pkt.adc2.mag_x =
        pack_i16(ut_x * MAG_PACKET_SCALE);

      env_adc2.pkt.adc2.mag_y =
        pack_i16(ut_y * MAG_PACKET_SCALE);

      env_adc2.pkt.adc2.mag_z =
        pack_i16(ut_z * MAG_PACKET_SCALE);
    }
  else
    {
      env_adc2.pkt.adc2.mag_x = 0;
      env_adc2.pkt.adc2.mag_y = 0;
      env_adc2.pkt.adc2.mag_z = 0;
    }

  env_adc2.pkt.adc2.footer =
    TELEM_FOOTER_ADC2;

  /*
   * Always store Beacon 2 locally.
   */

  telemetry_rb_write(&env_adc2);

  sem_post(&g_telemetry_sem);

  /**************************************************************************
   * 3. Extract live CW telemetry
   **************************************************************************/

  /* Beacon 1 live values */

  float v_batt_pin;
  enum sensor_status_e batt_status;

  float batt_v =
    convert_channel(adc0_raw[0],
                    &g_adc1_cfg[0],
                    &v_batt_pin,
                    &batt_status);

  uint16_t batt_v_int =
    (uint16_t)(batt_v * 100.0f);

  float v_total_solar_pin;
  enum sensor_status_e solar_status;

  float total_solar_v =
    convert_channel(adc0_raw[1],
                    &g_adc1_cfg[1],
                    &v_total_solar_pin,
                    &solar_status);

  uint16_t total_solar_v_int =
    (uint16_t)(total_solar_v * 100.0f);

  float ANT_temp_pin;
  enum sensor_status_e temp_status;

  float temp_c =
    convert_channel(adc0_raw[8],
                    &g_adc1_cfg[8],
                    &ANT_temp_pin,
                    &temp_status);

  uint16_t temp_c_int =
    (uint16_t)(temp_c * 10.0f);

  float BATT_temp_pin;
  enum sensor_status_e batt_temp_status;

  float batt_temp_c =
    convert_channel(adc0_raw[9],
                    &g_adc1_cfg[9],
                    &BATT_temp_pin,
                    &batt_temp_status);

  uint16_t batt_temp_c_int =
    (uint16_t)(batt_temp_c * 10.0f);

  float TEMP_BPB_pin;
  enum sensor_status_e bpb_temp_status;

  float bpb_temp_c =
    convert_channel(adc0_raw[10],
                    &g_adc1_cfg[10],
                    &TEMP_BPB_pin,
                    &bpb_temp_status);

  uint16_t bpb_temp_c_int =
    (uint16_t)(bpb_temp_c * 10.0f);

  /* Beacon 2 live values */

  float Main3v3_I_pin;
  enum sensor_status_e main3v3_i_status;

  float main3v3_i =
    convert_channel(adc1_raw[3],
                    &g_adc2_cfg[3],
                    &Main3v3_I_pin,
                    &main3v3_i_status);

  uint16_t main3v3_i_int =
    (uint16_t)(main3v3_i * 100.0f);

  float v_5v_i_pin;
  enum sensor_status_e v_5v_i_status;

  float v_5v_i =
    convert_channel(adc1_raw[8],
                    &g_adc2_cfg[8],
                    &v_5v_i_pin,
                    &v_5v_i_status);

  uint16_t v_5v_i_int =
    (uint16_t)(v_5v_i * 100.0f);

  float BAT_I_pin;
  enum sensor_status_e BAT_I_pin_status;

  float bat_i =
    convert_channel(adc1_raw[14],
                    &g_adc2_cfg[14],
                    &BAT_I_pin,
                    &BAT_I_pin_status);

  uint16_t bat_i_int =
    (uint16_t)(bat_i * 100.0f);

  int16_t flag1 = 1;
  int16_t flag2 = 2;

  /**************************************************************************
   * 4. Update M0+ live telemetry and notify through IPCC
   *
   * Existing protocol is intentionally preserved:
   *
   *   TELEM_ID_B1 updated
   *   TELEM_ID_B2 updated
   *   IPCC_CH_BEACON sent once
   *
   * scan_count is only used for logging here.
   **************************************************************************/

  pthread_mutex_lock(&g_beacon_mutex);

  printf("[OBC_MAIN] [TX -> M0+] Updating Beacon 1 & Beacon 2 live telemetry\n");

  live_telem_update_float(
      TELEM_ID_B1,
      (int16_t)batt_v_int,
      (int16_t)total_solar_v_int,
      (int16_t)temp_c_int,
      (int16_t)batt_temp_c_int,
      (int16_t)bpb_temp_c_int);
  printf("[OBC_MAIN] [TX -> M0+] Beacon 1 buffered (V_batt=%d.%02dV, Solar=%d.%02dV, T=%d.%dC)\n",
         batt_v_int / 100, (int)abs(batt_v_int % 100),
         total_solar_v_int / 100, (int)abs(total_solar_v_int % 100),
         temp_c_int / 10, (int)abs(temp_c_int % 10));

  live_telem_update_float(
      TELEM_ID_B2,
      (int16_t)main3v3_i_int,
      (int16_t)v_5v_i_int,
      (int16_t)bat_i_int,
      (int16_t)flag1,
      (int16_t)flag2);
  printf("[OBC_MAIN] [TX -> M0+] Beacon 2 buffered (I_3v3=%d.%02dA, I_5v=%d.%02dA, I_bat=%d.%02dA)\n",
         main3v3_i_int / 100, (int)abs(main3v3_i_int % 100),
         v_5v_i_int / 100, (int)abs(v_5v_i_int % 100),
         bat_i_int / 100, (int)abs(bat_i_int % 100));
  /*
   * Ring IPCC beacon doorbell.
   */

  ipcc_m4_send(IPCC_CH_BEACON);

  pthread_mutex_unlock(&g_beacon_mutex);

  /**************************************************************************
   * 5. Console CW line
   **************************************************************************/

  {
    int v_int;
    int v_frac;

    int t_int;
    int t_frac;

    v_int =
      (int)batt_v;

    v_frac =
      (int)((batt_v - (float)v_int) * 100.0f);

    if (v_frac < 0)
      {
        v_frac = -v_frac;
      }

    t_int =
      (int)temp_c;

    t_frac =
      (int)((temp_c - (float)t_int) * 10.0f);

    if (t_frac < 0)
      {
        t_frac = -t_frac;
      }

    printf("\n");
    printf("[OBC_MAIN] [TX -> M0+] CW Telemetry: "
           "\"9NS2S2 V%d.%02d T%d.%d\" | Cycle scan #%d\n",
           v_int,
           v_frac,
           t_int,
           t_frac,
           scan_count);
  }

  return 0;
}

/****************************************************************************
 * Name: check_camera_data_available
 ****************************************************************************/

static bool check_camera_data_available(void) __attribute__((unused));
static bool check_camera_data_available(void)
{
  DIR *d;

  d = opendir("/mnt/camera");

  if (!d)
    {
      return false;
    }

  {
    struct dirent *de;
    bool found = false;

    while ((de = readdir(d)) != NULL)
      {
        if (strcmp(de->d_name, ".") != 0 &&
            strcmp(de->d_name, "..") != 0)
          {
            struct stat st;
            char path[64];

            snprintf(path,
                     sizeof(path),
                     "/mnt/camera/%s",
                     de->d_name);

            if (stat(path, &st) == 0 &&
                st.st_size > 0)
              {
                found = true;
                break;
              }
          }
      }

    closedir(d);

    return found;
  }
}

#define CAM_UART_DEV              "/dev/ttyS1" /* LPUART1: PC0 (RX), PC1 (TX) */
#define CAM_FLASH_STORAGE_PATH    "/mnt/camera/capture_01.raw"
#define CAM_FLASH_STORAGE_PATH2   "/mnt/camera/capture_02.raw"

/* Real camera timing on LPUART1 (PC0 RX / PC1 TX):
 * 1. Send handshake command "CAM" and receive "CAM" back (handshake success).
 * 2. Send 0xCA command byte.
 * 3. First image wait: 30 seconds (CAM_SOI_WAIT_MS = 30000) for Image 1 SOI (0xFFD8).
 *    If nothing received in 30 seconds -> stop immediately.
 * 4. Second image wait: 30 seconds (CAM_SOI_WAIT_MS = 30000) AFTER Image 1 is received
 *    for Image 2 SOI (0xFFD8). If nothing received in 30 seconds -> stop.
 * 5. Post-EOI silence window: 500 ms to absorb repeated trailing 0xFFD9 markers
 *    (camera sends FFD9 multiple times, e.g., ~5 times) when Photo 2 does not
 *    immediately follow in the same burst.
 */
#define CAM_SOI_WAIT_MS           30000
#define CAM_POST_EOI_SILENCE_MS   500
#define CAM_IDLE_TIMEOUT_MS       30000
#define CAM_TOTAL_TIMEOUT_MS      75000

static uint32_t cam_elapsed_ms(const struct timespec *from)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint32_t)((now.tv_sec - from->tv_sec) * 1000 +
                    (now.tv_nsec - from->tv_nsec) / 1000000);
}

static void push_cam_chunk_to_rb(uint32_t frame_id, const uint8_t *data,
                                 uint16_t len, uint8_t is_last, uint8_t is_meta)
{
  struct image_chunk_s chunk;
  memset(&chunk, 0, sizeof(chunk));
  if (data != NULL && len > 0)
    {
      if (len > CHUNK_SIZE)
        {
          len = CHUNK_SIZE;
        }
      memcpy(chunk.data, data, len);
    }
  chunk.len      = len;
  chunk.frame_id = frame_id;
  chunk.is_last  = is_last;
  chunk.is_meta  = is_meta;

  /* Push burst chunk to ring buffer; let littlefs daemon (4KB stack) drain to Flash CAM */
  int retries = 100;
  while (rb_write(&chunk) != 0 && retries-- > 0)
    {
      sem_post(&g_telemetry_sem);
      usleep(5000);
    }

  if (retries < 0)
    {
      cam_rb_drain_to_flash();
      rb_write(&chunk);
    }

  /* Wake littlefs daemon only at batch threshold or end-of-frame to avoid console spam */
  if (is_last || rb_count() >= 8)
    {
      sem_post(&g_telemetry_sem);
    }
}

static void flush_cam_write_buf_to_rb(uint32_t frame_id, uint8_t *write_buf,
                                      size_t *write_buf_len, bool is_final)
{
  size_t off = 0;
  size_t total = *write_buf_len;

  if (total == 0 && is_final)
    {
      push_cam_chunk_to_rb(frame_id, NULL, 0, 1, 0);
      return;
    }

  while (off < total)
    {
      size_t rem = total - off;
      uint16_t clen = (rem > CHUNK_SIZE) ? CHUNK_SIZE : (uint16_t)rem;
      uint8_t last_flag = (is_final && (off + clen == total)) ? 1 : 0;
      push_cam_chunk_to_rb(frame_id, &write_buf[off], clen, last_flag, 0);
      off += clen;
    }

  *write_buf_len = 0;
}

/* Hardware registers for STM32WL55 LPUART1 on PC0 (RX, AF8) and PC1 (TX, AF8) */
#define WL55_RCC_AHB2ENR          (*(volatile uint32_t *)0x5800004CUL)
#define WL55_RCC_APB1ENR2         (*(volatile uint32_t *)0x5800005CUL)
#define WL55_RCC_CCIPR            (*(volatile uint32_t *)0x58000088UL)

#define WL55_GPIOC_MODER          (*(volatile uint32_t *)0x48000800UL)
#define WL55_GPIOC_OTYPER         (*(volatile uint32_t *)0x48000804UL)
#define WL55_GPIOC_OSPEEDR        (*(volatile uint32_t *)0x48000808UL)
#define WL55_GPIOC_PUPDR          (*(volatile uint32_t *)0x4800080CUL)
#define WL55_GPIOC_AFRL           (*(volatile uint32_t *)0x48000820UL)

#define WL55_LPUART1_CR1          (*(volatile uint32_t *)0x40008000UL)
#define WL55_LPUART1_CR2          (*(volatile uint32_t *)0x40008004UL)
#define WL55_LPUART1_CR3          (*(volatile uint32_t *)0x40008008UL)
#define WL55_LPUART1_BRR          (*(volatile uint32_t *)0x4000800CUL)
#define WL55_LPUART1_RQR          (*(volatile uint32_t *)0x40008018UL)
#define WL55_LPUART1_ISR          (*(volatile uint32_t *)0x4000801CUL)
#define WL55_LPUART1_ICR          (*(volatile uint32_t *)0x40008020UL)
#define WL55_LPUART1_PRESC        (*(volatile uint32_t *)0x4000802CUL)

static void lpuart1_pc0_pc1_hw_setup(void)
{
  /* 1. Enable GPIOC (AHB2 bit 2) and LPUART1 (APB1ENR2 bit 0) clocks;
   *    select PCLK1 (48 MHz, bits 11:10 = 00) for LPUART1 clock source.
   */
  WL55_RCC_AHB2ENR  |= (1UL << 2);
  WL55_RCC_APB1ENR2 |= (1UL << 0);
  WL55_RCC_CCIPR    &= ~(3UL << 10);

  /* 2. Configure PC0 = LPUART1_RX (AF8) and PC1 = LPUART1_TX (AF8):
   *    - MODER   : Alternate Function (10b) on PC0 (bits 1:0) & PC1 (bits 3:2)
   *    - OTYPER  : Push-Pull (0b) on PC0 & PC1
   *    - OSPEEDR : Very High Speed (11b) on PC0 & PC1
   *    - PUPDR   : Pull-Up (01b) on PC0 & PC1 so RX never floats
   *    - AFRL    : AF8 (1000b) on PC0 (bits 3:0) & PC1 (bits 7:4)
   */
  WL55_GPIOC_MODER   = (WL55_GPIOC_MODER   & ~0x0FUL) | 0x0AUL;
  WL55_GPIOC_OTYPER &= ~0x03UL;
  WL55_GPIOC_OSPEEDR |= 0x0FUL;
  WL55_GPIOC_PUPDR   = (WL55_GPIOC_PUPDR   & ~0x0FUL) | 0x05UL;
  WL55_GPIOC_AFRL    = (WL55_GPIOC_AFRL    & ~0xFFUL) | 0x88UL;

  /* 3. Unlock LPUART1 (UE=0) so BRR/CR1/CR2/CR3 can be written,
   *    preserve NuttX interrupt mask (RXNEIE), set 115200 8N1 (BRR=106667),
   *    enable OVRDIS (CR3 bit 12) so Overrun never blocks RX, flush RX, and enable.
   */
  uint32_t saved_ie = WL55_LPUART1_CR1 & 0x000001F0UL;
  WL55_LPUART1_CR1   = 0;
  WL55_LPUART1_CR2   = 0;
  WL55_LPUART1_CR3   = (1UL << 12); /* OVRDIS */
  WL55_LPUART1_PRESC = 0;
  WL55_LPUART1_BRR   = 106667UL;    /* (256 * 48,000,000) / 115,200 */
  WL55_LPUART1_RQR   = (1UL << 3);  /* RXFRQ: flush hardware RX */
  WL55_LPUART1_ICR   = 0xFFFFFFFFUL;
  WL55_LPUART1_CR1   = saved_ie | (1UL << 5) | (1UL << 3) | (1UL << 2) | (1UL << 0);
}

/****************************************************************************
 * Name: camera_capture_via_lpuart1
 *
 * Description:
 *   Turns on camera/ADCS power rails, opens LPUART1 (PC0 RX / PC1 TX,
 *   /dev/ttyS1 @ 115200 8N1 Raw), performs "MSN" -> "MSN" handshake with
 *   ADCS, and then:
 *     - When is_dummy == false (Real Camera Capture):
 *       Sends 0xCA trigger byte to ADCS and waits up to 30 seconds per image
 *       for up to two JPEG photos (FFD8 ... FFD9), storing into
 *       /mnt/camera/capture_01.raw and /mnt/camera/capture_02.raw.
 *     - When is_dummy == true (Dummy Camera Capture):
 *       Sends "CAM" command to ADCS and waits up to 30 seconds for the dummy
 *       JPEG photo (FFD8 ... FFD9) sent by ADCS, storing into the same Flash
 *       CAM partition (/mnt/camera/capture_01.raw).
 ****************************************************************************/

static size_t camera_capture_via_lpuart1(bool is_dummy)
{
  const int max_photos = 2; /* Both real and dummy ADCS modes send 2 JPEG images (FFD8..FFD9 x2) */
  g_camera_capture_active = true;
  rb_init();
  ensure_camera_partition_mounted();

  /* 1. First enable BIRDS Camera Power Rails:
   *    PA0:  5V DC/DC Power Rail EN
   *    PA12: 3.3V Logic Power Rail EN
   *    PC13: GPIO1 / Mission1 EN
   *    PC6:  GPIO2 / Mission2 EN
   */
  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [STEP 1: BIRDS CAMERA POWER ENABLE (%s)] Enabling GPIOs:\n"
         "            - PA0  : 5V DC/DC Power Rail -> ON\n"
         "            - PA12 : 3.3V Logic Power Rail -> ON\n"
         "            - PC13 : GPIO1 / Mission 1 Enable -> ON\n"
         "            - PC6  : GPIO2 / Mission 2 Enable -> ON\n"
         "[OBC_MAIN] ============================================================\n",
         is_dummy ? "DUMMY CAM VIA ADCS" : "REAL CAM VIA ADCS");
  board_camera_power(true);

  int fd = open(CAM_UART_DEV, O_RDWR | O_NOCTTY);
  if (fd < 0)
    {
      printf("[OBC_MAIN] [CAMERA LPUART1] Error: Could not open %s (errno=%d).\n",
             CAM_UART_DEV, errno);
      printf("[OBC_MAIN] >>> [CAMERA POWER OFF] Disabling PA0 (5V), PA12 (3.3V), PC13 (Mission1), PC6 (Mission2) <<<\n");
      board_camera_power(false);
      unlink(CAM_FLASH_STORAGE_PATH);
      unlink(CAM_FLASH_STORAGE_PATH2);
      g_camera_capture_active = false;
      return 0;
    }
  else
    {
      struct termios tio;
      if (tcgetattr(fd, &tio) == 0)
        {
          cfmakeraw(&tio);
          cfsetspeed(&tio, B115200);
          tio.c_cflag |= (CLOCAL | CREAD);
          tio.c_cflag &= ~PARENB;
          tio.c_cflag &= ~CSTOPB;
          tio.c_cflag &= ~CSIZE;
          tio.c_cflag |= CS8;
          tcsetattr(fd, TCSANOW, &tio);
        }

      /* Configure LPUART1 hardware on PC0 (RX, AF8) and PC1 (TX, AF8) @ 115200 8N1 */
      lpuart1_pc0_pc1_hw_setup();

      /* Allow 5V / 3.3V power rails and camera MCU to stabilize, then flush any power-up glitch bytes */
      usleep(200000); /* 200 ms power rail & camera boot stabilization */
      lpuart1_pc0_pc1_hw_setup();
      tcflush(fd, TCIOFLUSH);

      /* 2. Handshake: Send "MSN" command and verify "MSN" response */
      printf("[OBC_MAIN] >>> [STEP 2: ADCS/CAMERA HANDSHAKE] Sending 'MSN' handshake command on %s (LPUART1 PC0 RX / PC1 TX) <<<\n", CAM_UART_DEV);

      bool handshake_ok = false;
      const char msn_handshake_cmd[] = "MSN";
      int hs_attempts = 10; /* Up to 10 attempts, 500 ms each = 5s total */

      while (hs_attempts-- > 0 && !handshake_ok)
        {
          if (WL55_LPUART1_ISR & 0x0FUL)
            {
              WL55_LPUART1_ICR = 0x0FUL;
            }
          tcflush(fd, TCIOFLUSH);
          write(fd, msn_handshake_cmd, 3);
          printf("[OBC_MAIN] [CAMERA LPUART1] Sent 'MSN' handshake command (attempt %d/10)...\n",
                 10 - hs_attempts);

          struct timespec hs_start;
          clock_gettime(CLOCK_MONOTONIC, &hs_start);
          int match_idx = 0;

          while (cam_elapsed_ms(&hs_start) < 500)
            {
              if (WL55_LPUART1_ISR & 0x0FUL)
                {
                  WL55_LPUART1_ICR = 0x0FUL;
                }

              struct pollfd hs_pfd;
              hs_pfd.fd = fd;
              hs_pfd.events = POLLIN;

              if (poll(&hs_pfd, 1, 50) > 0 && (hs_pfd.revents & POLLIN))
                {
                  uint8_t ch = 0;
                  if (read(fd, &ch, 1) == 1)
                    {
                      if (match_idx == 0 && ch == 'M')
                        {
                          match_idx = 1;
                        }
                      else if (match_idx == 1 && ch == 'S')
                        {
                          match_idx = 2;
                        }
                      else if (match_idx == 2 && ch == 'N')
                        {
                          match_idx = 3;
                          handshake_ok = true;
                          break;
                        }
                      else
                        {
                          match_idx = (ch == 'M') ? 1 : 0;
                        }
                    }
                }
            }
        }

      if (!handshake_ok)
        {
          printf("[OBC_MAIN] [CAMERA LPUART1] ERROR: Camera handshake FAILED! Did not receive 'MSN' response.\n");
          printf("[OBC_MAIN] >>> [CAMERA POWER OFF] Disabling PA0 (5V), PA12 (3.3V), PC13 (Mission1), PC6 (Mission2) <<<\n");
          close(fd);
          board_camera_power(false);
          unlink(CAM_FLASH_STORAGE_PATH);
          unlink(CAM_FLASH_STORAGE_PATH2);
          g_camera_capture_active = false;
          return 0;
        }

      printf("[OBC_MAIN] [CAMERA LPUART1] >>> CAMERA HANDSHAKE SUCCESSFUL! Received 'MSN' from ADCS <<<\n");

      /* Allow ADCS MCU to prepare for command, then clear UART */
      usleep(50000);
      tcflush(fd, TCIOFLUSH);

      unlink(CAM_FLASH_STORAGE_PATH);
      unlink(CAM_FLASH_STORAGE_PATH2);

      /* 3. Send "CAM" (for Dummy Camera) or 0xCA (for Real Camera) over LPUART1 and enter receiving mode */
      if (is_dummy)
        {
          printf("[OBC_MAIN] >>> [STEP 3: SEND 'CAM' DUMMY COMMAND TO ADCS] LPUART1 (PC0 RX / PC1 TX) on %s <<<\n", CAM_UART_DEV);
          const char cam_cmd[3] = { 'C', 'A', 'M' };
          write(fd, cam_cmd, 3);
          printf("[OBC_MAIN] [CAMERA LPUART1] Sent 'CAM' dummy command to ADCS -> Entering RECEIVING MODE:\n"
                 "            - Dummy Image 1 wait: 30s (%d ms) -> %s\n"
                 "            - Dummy Image 2 wait: 30s (%d ms) after Image 1 -> %s\n"
                 "            - Simultaneous burst transfer -> RingBuffer & Flash CAM partition\n",
                 CAM_SOI_WAIT_MS, CAM_FLASH_STORAGE_PATH,
                 CAM_SOI_WAIT_MS, CAM_FLASH_STORAGE_PATH2);
        }
      else
        {
          printf("[OBC_MAIN] >>> [STEP 3: SEND 0xCA REAL CAM COMMAND TO ADCS] LPUART1 (PC0 RX / PC1 TX) on %s <<<\n", CAM_UART_DEV);
          const uint8_t trigger_byte = 0xCA;
          write(fd, &trigger_byte, 1);
          printf("[OBC_MAIN] [CAMERA LPUART1] Sent 0xCA command byte -> Entering RECEIVING MODE:\n"
                 "            - Image 1 wait: 30s (%d ms) -> %s\n"
                 "            - Image 2 wait: 30s (%d ms) after Image 1 -> %s\n"
                 "            - Simultaneous burst transfer -> RingBuffer & Flash CAM partition\n",
                 CAM_SOI_WAIT_MS, CAM_FLASH_STORAGE_PATH,
                 CAM_SOI_WAIT_MS, CAM_FLASH_STORAGE_PATH2);
        }
      printf("[OBC_MAIN] ============================================================\n");

      size_t photo1_len = 0;
      size_t photo2_len = 0;

      uint8_t rx_buf[256];
      uint8_t write_buf[512];
      size_t write_buf_len = 0;
      size_t eoi_write_buf_len = 0;
      size_t eoi_captured_len = 0;

      int current_photo = 1;
      bool in_photo = false;
      bool seen_eoi = false;
      unsigned int eoi_count = 0;
      size_t non_pad_after_eoi = 0;
      uint8_t prev_b = 0;
      size_t current_captured_len = 0;

      printf("[OBC_MAIN] [CAMERA LPUART1] Waiting up to %d ms for Photo 1 SOI (0xFFD8) -> %s\n",
             CAM_SOI_WAIT_MS, CAM_FLASH_STORAGE_PATH);

      struct pollfd pfd;
      pfd.fd = fd;
      pfd.events = POLLIN;

      struct timespec t_total_start;
      struct timespec t_photo_start;
      struct timespec t_last;
      struct timespec t_last_eoi;
      clock_gettime(CLOCK_MONOTONIC, &t_total_start);
      t_photo_start = t_total_start;
      t_last = t_total_start;
      t_last_eoi = t_total_start;

      bool adcs_nack = false;

      while (current_photo <= max_photos && !adcs_nack)
        {
          if (WL55_LPUART1_ISR & 0x0FUL)
            {
              WL55_LPUART1_ICR = 0x0FUL;
            }

          /* Overall session timeout (75s) */
          if (cam_elapsed_ms(&t_total_start) >= CAM_TOTAL_TIMEOUT_MS)
            {
              printf("[OBC_MAIN] [CAMERA LPUART1] Timeout: %d ms total capture window elapsed\n",
                     CAM_TOTAL_TIMEOUT_MS);
              break;
            }

          /* Waiting for SOI (0xFF 0xD8):
           * - Image 1: wait 30 seconds (CAM_SOI_WAIT_MS) after sending command.
           * - Image 2: wait 30 seconds (CAM_SOI_WAIT_MS) AFTER Image 1 was received.
           * If nothing is received within 30 seconds, stop immediately. */
          if (!in_photo && cam_elapsed_ms(&t_photo_start) >= CAM_SOI_WAIT_MS)
            {
              if (current_photo == 1)
                {
                  printf("[OBC_MAIN] [CAMERA LPUART1] Timeout: nothing received in %d ms (30s) for Image 1 -> Stopping!\n",
                         CAM_SOI_WAIT_MS);
                }
              else
                {
                  printf("[OBC_MAIN] [CAMERA LPUART1] Timeout: nothing received in %d ms (30s) after Image 1 for Image 2 -> Stopping!\n",
                         CAM_SOI_WAIT_MS);
                }
              break;
            }

          /* Post-EOI silence check: after seeing FFD9 (possibly repeated ~5 times),
           * if no new FFD8 or non-padding bytes arrive within 500 ms, finalize this photo. */
          if (in_photo && seen_eoi && non_pad_after_eoi == 0 &&
              cam_elapsed_ms(&t_last_eoi) >= CAM_POST_EOI_SILENCE_MS)
            {
              write_buf_len = eoi_write_buf_len;
              current_captured_len = eoi_captured_len;
              flush_cam_write_buf_to_rb((uint32_t)current_photo, write_buf, &write_buf_len, true);
              sem_post(&g_telemetry_sem);

              printf("[OBC_MAIN] [CAMERA LPUART1] >>> Photo %d: JPEG EOI (0xFFD9 x%u) COMPLETE! Total: %u B -> RingBuffer & Flash CAM <<<\n",
                     current_photo, eoi_count, (unsigned)current_captured_len);

              if (current_photo == 1)
                {
                  photo1_len = current_captured_len;
                  if (max_photos == 1)
                    {
                      current_photo = 2;
                      in_photo = false;
                      printf("[OBC_MAIN] [CAMERA LPUART1] Dummy Image stored (%u B) -> %s. Capture complete!\n",
                             (unsigned)photo1_len, CAM_FLASH_STORAGE_PATH);
                      break;
                    }
                  current_photo = 2;
                  in_photo = false;
                  seen_eoi = false;
                  eoi_count = 0;
                  non_pad_after_eoi = 0;
                  prev_b = 0;
                  /* Start fresh 30-second wait timer for Image 2 AFTER Image 1 is received */
                  clock_gettime(CLOCK_MONOTONIC, &t_photo_start);
                  t_last = t_photo_start;
                  printf("[OBC_MAIN] [CAMERA LPUART1] Image 1 stored (%u B). Now waiting 30s (%d ms) for Image 2 SOI (0xFFD8) -> %s\n",
                         (unsigned)photo1_len, CAM_SOI_WAIT_MS, CAM_FLASH_STORAGE_PATH2);
                  continue;
                }
              else
                {
                  photo2_len = current_captured_len;
                  current_photo = 3;
                  printf("[OBC_MAIN] [CAMERA LPUART1] Image 2 stored (%u B) -> %s. Both images received, stopping!\n",
                         (unsigned)photo2_len, CAM_FLASH_STORAGE_PATH2);
                  break;
                }
            }

          /* Inside photo data stream: silence timeout if camera stalls mid-image */
          if (in_photo && cam_elapsed_ms(&t_last) >= CAM_IDLE_TIMEOUT_MS)
            {
              printf("[OBC_MAIN] [CAMERA LPUART1] Timeout: camera silent for %d ms during Photo %d\n",
                     CAM_IDLE_TIMEOUT_MS, current_photo);
              break;
            }

          if (poll(&pfd, 1, 20) <= 0 || (pfd.revents & POLLIN) == 0)
            {
              if (rb_count() > 0)
                {
                  sem_post(&g_telemetry_sem);
                }
              continue;
            }

          ssize_t n = read(fd, rx_buf, sizeof(rx_buf));
          if (n > 0)
            {
              clock_gettime(CLOCK_MONOTONIC, &t_last);

              for (ssize_t i = 0; i < n; i++)
                {
                  uint8_t b = rx_buf[i];

                  if (!in_photo)
                    {
                      /* Detect explicit NACK (0xEE) from ADCS */
                      if (b == 0xEE && prev_b != 0xFF)
                        {
                          printf("[OBC_MAIN] [CAMERA LPUART1] Received NACK (0xEE) from ADCS -> Stopping!\n");
                          adcs_nack = true;
                          break;
                        }

                      /* Detect SOI: 0xFF followed by 0xD8 */
                      if (prev_b == 0xFF && b == 0xD8)
                        {
                          in_photo = true;
                          seen_eoi = false;
                          eoi_count = 0;
                          non_pad_after_eoi = 0;
                          current_captured_len = 2;
                          write_buf_len = 0;

                          /* Push start-of-frame marker to ring buffer so consumer opens/truncates file */
                          push_cam_chunk_to_rb((uint32_t)current_photo, NULL, 0, 0, 1);

                          write_buf[write_buf_len++] = 0xFF;
                          write_buf[write_buf_len++] = 0xD8;

                          const char *target_file = (current_photo == 1) ?
                                                    CAM_FLASH_STORAGE_PATH : CAM_FLASH_STORAGE_PATH2;
                          printf("[OBC_MAIN] [CAMERA LPUART1] >>> Photo %d: JPEG SOI (0xFFD8) DETECTED (waited %lu ms) -> %s <<<\n",
                                 current_photo, (unsigned long)cam_elapsed_ms(&t_photo_start), target_file);
                        }
                    }
                  else
                    {
                      /* Currently inside photo: append byte to local burst buffer */
                      write_buf[write_buf_len++] = b;
                      current_captured_len++;

                      if (!seen_eoi)
                        {
                          /* Keep last 2 bytes in write_buf so marker detection has full context */
                          if (write_buf_len >= (CHUNK_SIZE + 2))
                            {
                              push_cam_chunk_to_rb((uint32_t)current_photo, write_buf, CHUNK_SIZE, 0, 0);
                              write_buf_len -= CHUNK_SIZE;
                              memmove(write_buf, &write_buf[CHUNK_SIZE], write_buf_len);
                            }

                          /* Detect first JPEG EOI: 0xFF followed by 0xD9 */
                          if (prev_b == 0xFF && b == 0xD9)
                            {
                              seen_eoi = true;
                              eoi_count = 1;
                              non_pad_after_eoi = 0;
                              eoi_write_buf_len = write_buf_len;
                              eoi_captured_len = current_captured_len;
                              clock_gettime(CLOCK_MONOTONIC, &t_last_eoi);
                            }
                        }
                      else
                        {
                          /* Already saw at least one 0xFFD9 in this photo!
                           * Camera may send 0xFFD9 multiple times (~5 times) and then start
                           * the second image with 0xFFD8. */
                          if (prev_b == 0xFF && b == 0xD9)
                            {
                              eoi_count++;
                              clock_gettime(CLOCK_MONOTONIC, &t_last_eoi);
                              if (non_pad_after_eoi == 0)
                                {
                                  /* Repeated trailing 0xFFD9: collapse duplicate 0xFFD9 */
                                  write_buf_len = eoi_write_buf_len;
                                  current_captured_len = eoi_captured_len;
                                }
                              else
                                {
                                  /* Real data existed before this 0xFFD9: advance EOI position */
                                  eoi_write_buf_len = write_buf_len;
                                  eoi_captured_len = current_captured_len;
                                  non_pad_after_eoi = 0;
                                }
                            }
                          else if (prev_b == 0xFF && b == 0xD8)
                            {
                              /* Next image SOI (0xFFD8) arrived immediately after Photo 1's 0xFFD9s! */
                              write_buf_len = eoi_write_buf_len;
                              current_captured_len = eoi_captured_len;
                              flush_cam_write_buf_to_rb((uint32_t)current_photo, write_buf, &write_buf_len, true);
                              sem_post(&g_telemetry_sem);

                              printf("[OBC_MAIN] [CAMERA LPUART1] >>> Photo %d: JPEG EOI (0xFFD9 x%u) COMPLETE (%u B)! Next SOI (0xFFD8) detected <<<\n",
                                     current_photo, eoi_count, (unsigned)current_captured_len);

                              if (current_photo == 1 && max_photos >= 2)
                                {
                                  photo1_len = current_captured_len;
                                  current_photo = 2;
                                  in_photo = true;
                                  seen_eoi = false;
                                  eoi_count = 0;
                                  non_pad_after_eoi = 0;
                                  current_captured_len = 2;
                                  write_buf_len = 0;

                                  push_cam_chunk_to_rb(2, NULL, 0, 0, 1);
                                  write_buf[write_buf_len++] = 0xFF;
                                  write_buf[write_buf_len++] = 0xD8;

                                  clock_gettime(CLOCK_MONOTONIC, &t_photo_start);
                                  t_last = t_photo_start;
                                  printf("[OBC_MAIN] [CAMERA LPUART1] >>> Photo 2: JPEG SOI (0xFFD8) STARTED IMMEDIATELY -> %s <<<\n",
                                         CAM_FLASH_STORAGE_PATH2);
                                }
                              else
                                {
                                  if (current_photo == 1)
                                    {
                                      photo1_len = current_captured_len;
                                    }
                                  else
                                    {
                                      photo2_len = current_captured_len;
                                    }
                                  current_photo = max_photos + 1;
                                  in_photo = false;
                                  break;
                                }
                            }
                          else
                            {
                              if (b != 0xFF && b != 0x00 && b != 0xD9)
                                {
                                  non_pad_after_eoi++;
                                }

                              /* If real image data continues past an embedded 0xFFD9, resume normal streaming */
                              if (write_buf_len >= (CHUNK_SIZE + 16))
                                {
                                  if (non_pad_after_eoi > 0)
                                    {
                                      seen_eoi = false;
                                      non_pad_after_eoi = 0;
                                      push_cam_chunk_to_rb((uint32_t)current_photo, write_buf, CHUNK_SIZE, 0, 0);
                                      write_buf_len -= CHUNK_SIZE;
                                      memmove(write_buf, &write_buf[CHUNK_SIZE], write_buf_len);
                                    }
                                  else
                                    {
                                      /* Only padding after EOI: rewind to EOI */
                                      write_buf_len = eoi_write_buf_len;
                                      current_captured_len = eoi_captured_len;
                                    }
                                }
                            }
                        }
                    }

                  prev_b = b;
                }
            }
        }

      /* If loop exited while still in_photo, flush any captured data to ring buffer */
      if (in_photo && current_photo <= max_photos)
        {
          if (seen_eoi && non_pad_after_eoi == 0)
            {
              write_buf_len = eoi_write_buf_len;
              current_captured_len = eoi_captured_len;
            }
          flush_cam_write_buf_to_rb((uint32_t)current_photo, write_buf, &write_buf_len, true);
          if (current_photo == 1)
            {
              photo1_len = current_captured_len;
            }
          else if (current_photo == 2)
            {
              photo2_len = current_captured_len;
            }
        }

      close(fd);

      /* Ensure all ring buffer chunks are drained and synced to Flash CAM partition (/mnt/camera) */
      sem_post(&g_telemetry_sem);
      for (int wait_rb = 0; wait_rb < 500 && !rb_empty(); wait_rb++)
        {
          usleep(10000);
        }
      cam_rb_close_all_files();

      /* Mission finished: Disable Camera Power Rails */
      printf("[OBC_MAIN] >>> [CAMERA POWER OFF] Disabling PA0 (5V), PA12 (3.3V), PC13 (Mission1), PC6 (Mission2) <<<\n");
      board_camera_power(false);
      usleep(50000); /* 50 ms stabilization */
      g_camera_capture_active = false;

      struct stat st1;
      struct stat st2;
      long flash_sz1 = (stat(CAM_FLASH_STORAGE_PATH, &st1) == 0) ? (long)st1.st_size : 0;
      long flash_sz2 = (stat(CAM_FLASH_STORAGE_PATH2, &st2) == 0) ? (long)st2.st_size : 0;

      if (photo1_len >= 4 || flash_sz1 >= 4)
        {
          printf("[OBC_MAIN] [CAMERA LPUART1] Capture Complete: Photo 1 = %u B (%ld B in %s), Photo 2 = %u B (%ld B in %s)\n",
                 (unsigned)photo1_len, flash_sz1, CAM_FLASH_STORAGE_PATH,
                 (unsigned)photo2_len, flash_sz2, CAM_FLASH_STORAGE_PATH2);
          return photo1_len + photo2_len;
        }
      else if (photo2_len >= 4 || flash_sz2 >= 4)
        {
          printf("[OBC_MAIN] [CAMERA LPUART1] Capture Complete: Photo 2 = %u B (%ld B in %s)\n",
                 (unsigned)photo2_len, flash_sz2, CAM_FLASH_STORAGE_PATH2);
          return photo2_len;
        }

      printf("[OBC_MAIN] [CAMERA LPUART1] >>> NO CAMERA DATA ARRIVED WITHIN 30s TIMEOUT! <<<\n");
      printf("[OBC_MAIN] >>> Flash CAM partition left empty. Returning 0 bytes captured <<<\n\n");
      unlink(CAM_FLASH_STORAGE_PATH);
      unlink(CAM_FLASH_STORAGE_PATH2);
      return 0;
    }

  printf("[OBC_MAIN] [CAMERA LPUART1] UART open failed, cannot capture camera photo.\n");
  unlink(CAM_FLASH_STORAGE_PATH);
  unlink(CAM_FLASH_STORAGE_PATH2);
  g_camera_capture_active = false;
  return 0;
}

/****************************************************************************
 * Name: stream_real_camera_from_flash
 *
 * Description:
 *   Reads the stored JPEG photo (Real or Dummy) from the Flash CAM partition
 *   (/mnt/camera/capture_01.raw or /dev/camera) chunk-by-chunk (128 bytes
 *   per chunk) starting from start_addr, and sends each chunk to Cortex-M0+
 *   via shared SRAM2 ring buffer.
 ****************************************************************************/

static bool stream_real_camera_from_flash(uint32_t start_addr, uint16_t num_pkts)
{
  ensure_camera_partition_mounted();
  int cam_fd = -1;
  const char *selected_path = CAM_FLASH_STORAGE_PATH;

  /* If address offset is >= 0x80000, stream Photo 2 directly */
  if (start_addr >= 0x80000)
    {
      cam_fd = open(CAM_FLASH_STORAGE_PATH2, O_RDONLY);
      if (cam_fd >= 0)
        {
          off_t sz = lseek(cam_fd, 0, SEEK_END);
          if (sz <= 0)
            {
              close(cam_fd);
              cam_fd = -1;
            }
          else
            {
              start_addr -= 0x80000;
              selected_path = CAM_FLASH_STORAGE_PATH2;
            }
        }
    }

  /* Otherwise open Photo 1 (/mnt/camera/capture_01.raw) */
  if (cam_fd < 0)
    {
      cam_fd = open(CAM_FLASH_STORAGE_PATH, O_RDONLY);
      if (cam_fd >= 0)
        {
          off_t sz = lseek(cam_fd, 0, SEEK_END);
          if (sz <= 0)
            {
              close(cam_fd);
              cam_fd = -1;
            }
          else
            {
              selected_path = CAM_FLASH_STORAGE_PATH;
            }
        }
    }

  /* Fallback to Photo 2 (/mnt/camera/capture_02.raw) if Photo 1 not found or empty */
  if (cam_fd < 0)
    {
      cam_fd = open(CAM_FLASH_STORAGE_PATH2, O_RDONLY);
      if (cam_fd >= 0)
        {
          off_t sz = lseek(cam_fd, 0, SEEK_END);
          if (sz <= 0)
            {
              close(cam_fd);
              cam_fd = -1;
            }
          else
            {
              selected_path = CAM_FLASH_STORAGE_PATH2;
            }
        }
    }

  off_t file_size = 0;
  uint8_t hdr[2] = { 0, 0 };
  bool valid_jpeg_hdr = false;
  if (cam_fd >= 0)
    {
      file_size = lseek(cam_fd, 0, SEEK_END);
      if (file_size >= 4)
        {
          lseek(cam_fd, 0, SEEK_SET);
          if (read(cam_fd, hdr, 2) == 2 && hdr[0] == 0xFF && hdr[1] == 0xD8)
            {
              valid_jpeg_hdr = true;
            }
        }
      lseek(cam_fd, start_addr, SEEK_SET);
    }

  /* Check if photo data in Flash is null / empty / invalid */
  if (cam_fd < 0 || file_size < 4 || !valid_jpeg_hdr || (uint32_t)file_size <= start_addr)
    {
      printf("\n[OBC_MAIN] ============================================================\n");
      printf("[OBC_MAIN] >>> [CAMERA FLASH IPC] REJECTED: FLASH PHOTO DATA IS NULL / EMPTY! <<<\n");
      printf("[OBC_MAIN]     cam_fd=%d, size=%ld B, valid_soi=%d, start_addr=0x%08lX\n",
             cam_fd, (long)file_size, (int)valid_jpeg_hdr, (unsigned long)start_addr);
      printf("[OBC_MAIN]     No valid photo in Flash CAM partition -> Rejecting & leaving ring buffer empty for M0+ NACK\n");
      printf("[OBC_MAIN] ============================================================\n\n");
      if (cam_fd >= 0)
        {
          close(cam_fd);
        }
      return false;
    }

  uint16_t avail_chunks = (uint16_t)((file_size - start_addr + CAM_CHUNK_SIZE - 1) / CAM_CHUNK_SIZE);
  uint16_t num_chunks = num_pkts;
  if (num_chunks == 0 || num_chunks > avail_chunks)
    {
      num_chunks = avail_chunks;
    }

  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [CAMERA FLASH IPC] STREAMING PHOTO (REAL/DUMMY) FROM CAM PARTITION (%s) <<<\n",
         selected_path);
  printf("[OBC_MAIN]     fd=%d (size=%ld B), start=0x%08lX, %u chunks of 128 B -> M0+\n",
         cam_fd, (long)file_size, (unsigned long)start_addr, (unsigned)num_chunks);
  printf("[OBC_MAIN] Transport: Shared SRAM2 Ring Buffer + IPCC_CH_FLASH\n");
  printf("[OBC_MAIN] ============================================================\n");

  for (uint16_t p = 0; p < num_chunks; p++)
    {
      struct flash_chunk_s chunk;
      memset(&chunk, 0, sizeof(chunk));
      chunk.flash_addr = start_addr + (uint32_t)p * CAM_CHUNK_SIZE;
      chunk.pkt_idx    = p;
      chunk.total_pkts = num_chunks;
      chunk.data_len   = CAM_CHUNK_SIZE;
      chunk.status     = (cam_fd >= 0) ? 0 : 1;

      ssize_t n = 0;
      if (cam_fd >= 0)
        {
          n = read(cam_fd, chunk.data, CAM_CHUNK_SIZE);
        }
      if (n <= 0)
        {
          printf("[OBC_MAIN] [CAMERA FLASH IPC] Read returned %zd B (null) on chunk %u -> Stopping!\n",
                 n, p + 1);
          break;
        }
      if (n < (ssize_t)CAM_CHUNK_SIZE)
        {
          memset(&chunk.data[n], 0x00, CAM_CHUNK_SIZE - n);
        }

      /* Wait up to 15 seconds for M0+ to read each chunk from ring buffer */
      int retries = 3000;
      while (!flash_data_write(&chunk) && retries-- > 0)
        {
          usleep(5000);
        }

      if (retries <= 0)
        {
          printf("[OBC_MAIN] WARNING: Ring buffer full, M0+ timeout on cam chunk %u/%u\n",
                 p + 1, (unsigned)num_chunks);
          break;
        }

      ipcc_m4_send(IPCC_CH_FLASH);
      printf("[OBC_MAIN] [CAM IPC TX] Chunk %u/%u (128B, addr 0x%04lX, read %zd B) -> IPCC_CH_FLASH signaled\n",
             p + 1, (unsigned)num_chunks, (unsigned long)chunk.flash_addr, n);

      usleep(10000);
    }

  if (cam_fd >= 0)
    {
      close(cam_fd);
    }

  printf("[OBC_MAIN] >>> [CAMERA FLASH IPC] ALL %u CHUNKS STREAMED TO M0+ <<<\n\n",
         (unsigned)num_chunks);
  return true;
}

/****************************************************************************
 * Name: stream_hk_from_flash
 *
 * Description:
 *   Reads stored HK records from Flash (/mnt/hk/telemetry.bin or /dev/hk)
 *   chunk-by-chunk (128 bytes per chunk) starting from start_addr, and sends
 *   each chunk to Cortex-M0+ via shared SRAM2 ring buffer.
 *   If flash is null / empty / uninitialized, rejects the read and returns false.
 ****************************************************************************/

static bool stream_hk_from_flash(uint32_t start_addr, uint16_t num_pkts)
{
  if (num_pkts == 0) num_pkts = 1;

  ensure_hk_partition_mounted();
  hk_rb_drain_to_flash();

  int fd = open("/mnt/hk/telemetry.bin", O_RDONLY);
  off_t file_size = 0;
  if (fd >= 0)
    {
      file_size = lseek(fd, 0, SEEK_END);
    }

  if (fd < 0 || file_size < (off_t)FLASH_DATA_LEN)
    {
      printf("\n[OBC_MAIN] ============================================================\n");
      printf("[OBC_MAIN] >>> [HK FLASH IPC] REJECTED: HK FLASH DATA IS NULL / EMPTY! <<<\n");
      printf("[OBC_MAIN]     fd=%d, size=%ld B, start_addr=0x%08lX\n",
             fd, (long)file_size, (unsigned long)start_addr);
      printf("[OBC_MAIN]     No HK data in Flash -> Rejecting & leaving ring buffer empty for M0+ NACK\n");
      printf("[OBC_MAIN] ============================================================\n\n");
      if (fd >= 0)
        {
          close(fd);
        }
      return false;
    }

  uint32_t avail_records = (uint32_t)(file_size / FLASH_DATA_LEN);
  uint32_t start_rec     = start_addr / FLASH_DATA_LEN;

  printf("\n[OBC_MAIN] ============================================================\n");
  printf("[OBC_MAIN] >>> [HK FLASH IPC] STREAMING HK TELEMETRY FROM FLASH (/mnt/hk/telemetry.bin) <<<\n");
  printf("[OBC_MAIN]     fd=%d (size=%ld B, %lu stored records), start=0x%08lX, %u chunks of %u B -> M0+\n",
         fd, (long)file_size, (unsigned long)avail_records,
         (unsigned long)start_addr, (unsigned)num_pkts, (unsigned)FLASH_DATA_LEN);
  printf("[OBC_MAIN] Transport: Shared SRAM2 Ring Buffer + IPCC_CH_FLASH\n");
  printf("[OBC_MAIN] ============================================================\n");

  uint16_t chunks_sent = 0;

  for (uint16_t p = 0; p < num_pkts; p++)
    {
      uint32_t rec_idx = (start_rec + p) % avail_records;
      uint32_t read_offset = rec_idx * FLASH_DATA_LEN;

      struct flash_chunk_s chunk;
      memset(&chunk, 0, sizeof(chunk));
      chunk.flash_addr = start_addr + (uint32_t)p * FLASH_DATA_LEN;
      chunk.pkt_idx    = p;
      chunk.total_pkts = num_pkts;
      chunk.data_len   = FLASH_DATA_LEN;
      chunk.status     = 0;

      lseek(fd, (off_t)read_offset, SEEK_SET);
      ssize_t n = read(fd, chunk.data, FLASH_DATA_LEN);
      if (n <= 0)
        {
          break;
        }
      if (n < (ssize_t)FLASH_DATA_LEN)
        {
          memset(&chunk.data[n], 0x00, FLASH_DATA_LEN - n);
        }

      /* Reject if chunk is completely blank (all 0x00 or all 0xFF) */
      bool all_zero = true;
      bool all_ff   = true;
      for (ssize_t bi = 0; bi < n; bi++)
        {
          if (chunk.data[bi] != 0x00) all_zero = false;
          if (chunk.data[bi] != 0xFF) all_ff   = false;
        }
      if (all_zero || all_ff)
        {
          printf("[OBC_MAIN] [HK FLASH IPC] Chunk %u is null/blank (all 0x00/0xFF) -> Rejecting!\n",
                 p + 1);
          break;
        }

      /* Wait up to 15 seconds per chunk for M0+ to drain SRAM2 ring buffer */
      int retries = 3000;
      while (!flash_data_write(&chunk) && retries-- > 0)
        {
          usleep(5000);
        }

      if (retries <= 0)
        {
          printf("[OBC_MAIN] WARNING: Ring buffer full on HK chunk %u/%u\n",
                 p + 1, (unsigned)num_pkts);
          break;
        }

      chunks_sent++;
      ipcc_m4_send(IPCC_CH_FLASH);
      printf("[OBC_MAIN] [HK IPC TX] Chunk %u/%u (128B, flash_rec=%lu, addr=0x%04lX) -> IPCC_CH_FLASH signaled\n",
             p + 1, (unsigned)num_pkts, (unsigned long)rec_idx, (unsigned long)chunk.flash_addr);
      usleep(10000);
    }

  if (fd >= 0)
    {
      close(fd);
    }

  if (chunks_sent == 0)
    {
      printf("[OBC_MAIN] >>> [HK FLASH IPC] REJECTED: 0 VALID CHUNKS READ FROM FLASH <<<\n\n");
      return false;
    }

  printf("[OBC_MAIN] >>> [HK FLASH IPC] %u/%u CHUNKS STREAMED TO M0+ <<<\n\n",
         (unsigned)chunks_sent, (unsigned)num_pkts);
  return true;
}

/****************************************************************************
 * Name: process_ground_commands
 ****************************************************************************/

static void process_ground_commands(void)
{
  if (ipcc_m4_received(IPCC_CH_FLASH))
    {
      ipcc_m4_clear(IPCC_CH_FLASH);
      if (SHARED_FLASH_REQ->magic == FLASH_REQ_MAGIC)
        {
          uint32_t addr = SHARED_FLASH_REQ->start_addr;
          uint16_t num_pkts = SHARED_FLASH_REQ->num_chunks;
          if (num_pkts == 0) num_pkts = 1;

          if (num_pkts == CAM_NUM_CHUNKS || num_pkts >= 10 || addr == 0x00080000)
            {
              printf("[OBC_MAIN] IPCC_CH_FLASH Camera Image Request: addr=0x%08lX pkts=%u\n", (unsigned long)addr, num_pkts);
              stream_real_camera_from_flash(addr, num_pkts);
              return;
            }

          printf("[OBC_MAIN] IPCC_CH_FLASH Direct HK Request: addr=0x%08lX pkts=%u\n",
                 (unsigned long)addr, num_pkts);
          stream_hk_from_flash(addr, num_pkts);
        }
    }

  if (!ipcc_m4_received(IPCC_CH_COMMAND) &&
      rb_rx_empty())
    {
      return;
    }

  {
    struct rx_command_s rx_cmd;

    if (rb_rx_read(&rx_cmd))
      {
        ipcc_m4_clear(IPCC_CH_COMMAND);

        printf("\n[OBC_MAIN] [GROUND COMMAND RECEIVED]: ");

        bool cmd_all_null = true;
        for (int i = 0; i < CMD_PAYLOAD_LEN; i++)
          {
            printf("%02X ", rx_cmd.cmd[i]);
            if (rx_cmd.cmd[i] != 0x00)
              {
                cmd_all_null = false;
              }
          }

        printf("\n");

        if (cmd_all_null || rx_cmd.cmd[0] != 0x53)
          {
            printf("[OBC_MAIN] REJECTED: Null or invalid telecommand (signature 0x%02X != 0x53)\n",
                   rx_cmd.cmd[0]);
            SHARED_FLASH_REQ->seq = 0;
            ipcc_m4_send(IPCC_CH_COMMAND);
            return;
          }

        /********************************************************************
         * Dummy Camera Capture Command: 53 04 CC 5E BE ... (or data_id == 0x00D0)
         * or 8-byte format: 53 00 C5 E0 ...
         ********************************************************************/

        bool is_dummy_cam = ((rx_cmd.cmd[1] == CMD_TYPE_CAMERA &&
                              ((rx_cmd.cmd[2] == 0xCC && (rx_cmd.cmd[3] == 0x5E || rx_cmd.cmd[3] == 0x58) && rx_cmd.cmd[4] == 0xBE) ||
                               (rx_cmd.cmd[2] == 0xCC && rx_cmd.cmd[4] == 0xBE) ||
                               (rx_cmd.cmd[5] == 0x00 && rx_cmd.cmd[6] == 0xD0))) ||
                             (rx_cmd.cmd[2] == 0xC5 && rx_cmd.cmd[3] == 0xE0));

        /********************************************************************
         * Camera Data Downlink Command (Real & Dummy stored in Flash CAM):
         *   53 04 CC 5E BD 00 00 00 00 00 00 00 00
         * Or legacy OBC download:
         *   53 01 1D D2 F5 [DATA_ID:2B] [ADDR:4B] [PKTS:2B]
         ********************************************************************/

        bool is_cam_down = ((rx_cmd.cmd[1] == CMD_TYPE_CAMERA &&
                             rx_cmd.cmd[2] == 0xCC &&
                             (rx_cmd.cmd[3] == 0x5E || rx_cmd.cmd[3] == 0x58) &&
                             rx_cmd.cmd[4] == 0xBD) ||
                            (rx_cmd.cmd[1] == 0x01 && rx_cmd.cmd[2] == 0x1D &&
                             rx_cmd.cmd[3] == 0xD2 && rx_cmd.cmd[4] == 0xF5));

        /********************************************************************
         * Real Camera Capture Command (store to flash, NO downlink):
         *   53 04 CC 5E BC ... or 53 04 CC 5E CA ... or 53 04 CA ...
         ********************************************************************/

        bool is_real_cam_cap = (rx_cmd.cmd[1] == CMD_TYPE_CAMERA &&
                                ((rx_cmd.cmd[2] == 0xCC && (rx_cmd.cmd[4] == 0xBC || rx_cmd.cmd[4] == 0xCA)) ||
                                 rx_cmd.cmd[2] == 0xCA));

        if (is_dummy_cam)
          {
            printf("[OBC_MAIN] Detected Verified DUMMY CAMERA CAPTURE Command -> Handshake 'MSN', send 'CAM' to ADCS, receive Dummy JPEG (FFD8..FFD9) and store in Flash CAM partition (NO downlink)\n");
            size_t cap_len = camera_capture_via_lpuart1(true);
            if (cap_len > 0)
              {
                printf("[OBC_MAIN] Dummy camera capture from ADCS and flash storage complete (%u bytes) -> Signaling M0+ SUCCESS\n",
                       (unsigned)cap_len);
                SHARED_FLASH_REQ->seq = 1;
              }
            else
              {
                printf("[OBC_MAIN] Dummy camera capture from ADCS FAILED -> Signaling M0+ NACK\n");
                SHARED_FLASH_REQ->seq = 0;
              }
            ipcc_m4_send(IPCC_CH_COMMAND); /* Signal M0+ that capture task is complete */
          }
        else if (is_cam_down)
          {
            uint32_t cam_addr = ((uint32_t)rx_cmd.cmd[7]  << 24) |
                                ((uint32_t)rx_cmd.cmd[8]  << 16) |
                                ((uint32_t)rx_cmd.cmd[9]  <<  8) |
                                 (uint32_t)rx_cmd.cmd[10];
            uint16_t cam_pkts = ((uint16_t)rx_cmd.cmd[11] <<  8) | rx_cmd.cmd[12];
            printf("[OBC_MAIN] Detected Verified CAMERA DATA DOWNLOAD Command (53 04 CC 5E BD...): addr=0x%08lX pkts=%u -> Checking Flash CAM partition...\n",
                   (unsigned long)cam_addr, cam_pkts);
            bool has_data = stream_real_camera_from_flash(cam_addr, cam_pkts);
            if (has_data)
              {
                printf("[OBC_MAIN] Camera photo (Real/Dummy) streamed from Flash CAM partition to ring buffer -> Notifying M0+\n");
              }
            else
              {
                printf("[OBC_MAIN] Camera photo data null/empty in Flash CAM partition -> Rejecting & leaving ring buffer empty for M0+ NACK\n");
              }
            ipcc_m4_send(IPCC_CH_COMMAND); /* Notify M0+ command processing complete */
          }
        else if (is_real_cam_cap)
          {
            printf("[OBC_MAIN] Detected Verified REAL CAMERA CAPTURE Command -> Handshake 'MSN', 0xCA trigger, wait 30s receiving mode for 2 Photos (FFD8..FFD9) to Flash CAM partition (NO downlink)\n");
            size_t cap_len = camera_capture_via_lpuart1(false);
            if (cap_len > 0)
              {
                printf("[OBC_MAIN] Camera capture and flash storage complete (%u bytes) -> Signaling M0+ SUCCESS\n",
                       (unsigned)cap_len);
                SHARED_FLASH_REQ->seq = 1;
              }
            else
              {
                printf("[OBC_MAIN] Camera capture FAILED (handshake failed or no data arrived in 30s) -> Signaling M0+ NACK\n");
                SHARED_FLASH_REQ->seq = 0;
              }
            ipcc_m4_send(IPCC_CH_COMMAND); /* Signal M0+ that capture task is complete */
          }

        /********************************************************************
         * Housekeeping / Flash Download commands (from Packet Format.xlsx):
         *   HK Download   = 53 01 1D D1 F2 [DID:2B] [ADDR:4B] [PKTS:2B] (sat_health.txt)
         *   Flash Read    = 53 01 1D D1 F8 [DID:2B] [ADDR:4B] [PKTS:2B] (Flash Storage)
         *   Ping / Beacon = 53 01 1D D1 F5 / D2 F5 ...
         ********************************************************************/

        else if ((rx_cmd.cmd[1] == CMD_TYPE_HK && rx_cmd.cmd[2] == 0x1D &&
                  (rx_cmd.cmd[4] == 0xF2 || rx_cmd.cmd[4] == 0xF8)) ||
                 rx_cmd.cmd[4] == 0xF8)
          {
            uint32_t addr = ((uint32_t)rx_cmd.cmd[7]  << 24) |
                            ((uint32_t)rx_cmd.cmd[8]  << 16) |
                            ((uint32_t)rx_cmd.cmd[9]  <<  8) |
                             (uint32_t)rx_cmd.cmd[10];
            uint16_t num_pkts = ((uint16_t)rx_cmd.cmd[11] << 8) | (uint16_t)rx_cmd.cmd[12];
            if (num_pkts == 0) num_pkts = 1;

            /* Live HK (count 0/1): refresh beacon only. Do not stream N flash
             * chunks — that raced M0+ (one 128 B live packet) and filled the ring. */
            if (rx_cmd.cmd[4] == 0xF2 && num_pkts <= 1)
              {
                printf("[OBC_MAIN] Detected LIVE HK (1D D1 F2, pkts=%u) -> beacon telem only\n",
                       (unsigned)num_pkts);
                pthread_mutex_lock(&g_beacon_mutex);
                ipcc_m4_send(IPCC_CH_BEACON);
                pthread_mutex_unlock(&g_beacon_mutex);
                ipcc_m4_send(IPCC_CH_COMMAND);
              }
            else
              {
                const char *cmd_type_str = (rx_cmd.cmd[4] == 0xF8) ? "FLASH READ (1D D1 F8)" : "HK DOWNLOAD (1D D1 F2)";
                printf("[OBC_MAIN] Detected %s: addr=0x%08lX pkts=%u -> Streaming %u chunks to M0+\n",
                       cmd_type_str, (unsigned long)addr, (unsigned)num_pkts, (unsigned)num_pkts);

                if (rx_cmd.cmd[4] == 0xF2)
                  {
                    pthread_mutex_lock(&g_beacon_mutex);
                    ipcc_m4_send(IPCC_CH_BEACON);
                    pthread_mutex_unlock(&g_beacon_mutex);
                  }

                bool has_data = stream_hk_from_flash(addr, num_pkts);
                if (has_data)
                  {
                    printf("[OBC_MAIN] %s chunks completed -> Notifying M0+ via IPCC\n", cmd_type_str);
                  }
                else
                  {
                    printf("[OBC_MAIN] %s data null/empty -> Rejecting & leaving ring buffer empty for M0+ NACK\n", cmd_type_str);
                  }
                ipcc_m4_send(IPCC_CH_COMMAND);
              }
          }
        else if (rx_cmd.cmd[1] == CMD_TYPE_HK &&
                 rx_cmd.cmd[2] == 0x1D &&
                 (rx_cmd.cmd[4] == 0xF5))
          {
            printf("[OBC_MAIN] Detected Verified HK Ping Command (1D D1/D2 F5)\n");
            pthread_mutex_lock(&g_beacon_mutex);
            ipcc_m4_send(IPCC_CH_BEACON);
            pthread_mutex_unlock(&g_beacon_mutex);
            ipcc_m4_send(IPCC_CH_COMMAND); /* Signal M0+ task complete */
          }

        /********************************************************************
         * Unknown / non-matching command -> REJECT
         ********************************************************************/

        else
          {
            printf("[OBC_MAIN] REJECTED: Unrecognized telecommand (Opcode bytes: %02X %02X %02X %02X)\n",
                   rx_cmd.cmd[1], rx_cmd.cmd[2], rx_cmd.cmd[3], rx_cmd.cmd[4]);
            SHARED_FLASH_REQ->seq = 0;
            ipcc_m4_send(IPCC_CH_COMMAND); /* Signal M0+ task complete */
          }
      }
  }
}

/****************************************************************************
 * Name: display_adc1_telemetry
 ****************************************************************************/

static void display_adc1_telemetry(
                         uint16_t raw_data[ADS7953_CHANNELS])
{
  int ch;

  printf("\n");
  printf("================================ ADC 1: VOLTAGE & TEMPERATURE "
         "===============================\n");

  printf("  CH  | Signal Name       | Type        | Raw   | Pin Volt | "
         "Physical Value   | Status        \n");

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  for (ch = 0;
       ch < ADS7953_CHANNELS;
       ch++)
    {
      const struct adc_channel_cfg_s *cfg =
        &g_adc1_cfg[ch];

      float v_pin;
      float phys_val;

      enum sensor_status_e status;

      const char *type_str;
      const char *status_str;

      if (cfg->is_nc)
        {
          printf("  %02d  | %-17s | NC (Unused) |  ---  |   ----   | "
                 "       ---       | [ NC (Unused) ]\n",
                 ch,
                 cfg->name);

          continue;
        }

      phys_val =
        convert_channel(raw_data[ch],
                        cfg,
                        &v_pin,
                        &status);

      type_str =
        (cfg->type == CHANNEL_TYPE_TEMP) ?
        "Temperature" :
        "Voltage";

      switch (status)
        {
          case STATUS_ACTIVE_OK:
            status_str = "ACTIVE (OK)";
            break;

          case STATUS_DISCONNECTED:
            status_str = "[ DISCONNECTED ]";
            break;

          case STATUS_NO_DATA_DARK:
            status_str = "[ NO DATA/DARK ]";
            break;

          default:
            status_str = "---";
            break;
        }

      if (status == STATUS_DISCONNECTED)
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V | "
                 "      ---        | %-15s\n",
                 ch,
                 cfg->name,
                 type_str,
                 raw_data[ch],
                 v_pin,
                 status_str);
        }
      else if (status == STATUS_NO_DATA_DARK)
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V | "
                 "  0.000 %-4s     | %-15s\n",
                 ch,
                 cfg->name,
                 type_str,
                 raw_data[ch],
                 v_pin,
                 cfg->unit,
                 status_str);
        }
      else
        {
          printf("  %02d  | %-17s | %-11s | %5u | %6.3f V | "
                 "%9.3f %-4s   | %-15s\n",
                 ch,
                 cfg->name,
                 type_str,
                 raw_data[ch],
                 v_pin,
                 phys_val,
                 cfg->unit,
                 status_str);
        }
    }

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: display_adc2_telemetry
 ****************************************************************************/

static void display_adc2_telemetry(
                         uint16_t raw_data[ADS7953_CHANNELS],
                         const struct iam20380_data_s *gyro,
                         const struct mmc5983ma_data_s *mag,
                         bool gyro_ok,
                         bool mag_ok)
{
  int ch;

  printf("\n");
  printf("=================================== ADC 2: CURRENT SENSORS + IMU "
         "=============================\n");

  printf("  CH  | Signal Name       | Type        | Raw   | Pin Volt | "
         "Current Value    | Status        \n");

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  /**************************************************************************
   * Current channels
   **************************************************************************/

  for (ch = 0;
       ch < ADS7953_CHANNELS;
       ch++)
    {
      const struct adc_channel_cfg_s *cfg =
        &g_adc2_cfg[ch];

      float v_pin;
      float phys_val;

      enum sensor_status_e status;

      const char *status_str;

      if (cfg->is_nc)
        {
          continue;
        }

      phys_val =
        convert_channel(raw_data[ch],
                        cfg,
                        &v_pin,
                        &status);

      switch (status)
        {
          case STATUS_ACTIVE_OK:
            status_str = "ACTIVE (OK)";
            break;

          case STATUS_NO_DATA_DARK:
            status_str = "[ DISCONNECTED ]";
            break;

          case STATUS_IDLE_NO_LOAD:
            status_str = "[ IDLE/NO LOAD ]";
            break;

          default:
            status_str = "---";
            break;
        }

      if (status == STATUS_NO_DATA_DARK ||
          status == STATUS_IDLE_NO_LOAD)
        {
          printf("  %02d  | %-17s | Current     | %5u | %6.3f V | "
                 "  0.000 %-4s     | %-15s\n",
                 ch,
                 cfg->name,
                 raw_data[ch],
                 v_pin,
                 cfg->unit,
                 status_str);
        }
      else
        {
          printf("  %02d  | %-17s | Current     | %5u | %6.3f V | "
                 "%9.3f %-4s   | %-15s\n",
                 ch,
                 cfg->name,
                 raw_data[ch],
                 v_pin,
                 phys_val,
                 cfg->unit,
                 status_str);
        }
    }

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  /**************************************************************************
   * Gyroscope
   **************************************************************************/

  if (gyro_ok && gyro != NULL)
    {
      float dps_x =
        iam20380_counts_to_dps(gyro->x);

      float dps_y =
        iam20380_counts_to_dps(gyro->y);

      float dps_z =
        iam20380_counts_to_dps(gyro->z);

      float temp_c =
        iam20380_counts_to_degc(gyro->temp);

      printf("  GX  | %-17s | Gyro        | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->x,
             dps_x,
             "d/s",
             "ACTIVE (OK)");

      printf("  GY  | %-17s | Gyro        | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->y,
             dps_y,
             "d/s",
             "ACTIVE (OK)");

      printf("  GZ  | %-17s | Gyro        | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->z,
             dps_z,
             "d/s",
             "ACTIVE (OK)");

      printf("  GT  | %-17s | Gyro Temp   | %5d |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "IAM20380_GYRO",
             gyro->temp,
             temp_c,
             "C",
             "ACTIVE (OK)");
    }
  else
    {
      printf("  GYR | %-17s | Gyro        |  ---  |   ----   | "
             "      ---       | %-15s\n",
             "IAM20380_GYRO",
             "[ DISCONNECTED ]");
    }

  printf("------+-------------------+-------------+-------+----------+"
         "------------------+---------------\n");

  /**************************************************************************
   * Magnetometer
   **************************************************************************/

  if (mag_ok && mag != NULL)
    {
      float ut_x =
        mmc5983ma_counts_to_ut(mag->x);

      float ut_y =
        mmc5983ma_counts_to_ut(mag->y);

      float ut_z =
        mmc5983ma_counts_to_ut(mag->z);

      float ut_mag =
        sqrtf((ut_x * ut_x) +
              (ut_y * ut_y) +
              (ut_z * ut_z));

      const char *mag_status =
        (ut_mag > 20.0f &&
         ut_mag < 70.0f) ?
        "ACTIVE (OK)" :
        "[ CHECK HIRON ]";

      printf("  MX  | %-17s | Mag         | %5ld |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             (long)mag->x,
             ut_x,
             "uT",
             mag_status);

      printf("  MY  | %-17s | Mag         | %5ld |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             (long)mag->y,
             ut_y,
             "uT",
             mag_status);

      printf("  MZ  | %-17s | Mag         | %5ld |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             (long)mag->z,
             ut_z,
             "uT",
             mag_status);

      printf("  M|B|| %-17s | Mag |B|     |  ---  |   ----   | "
             "%9.3f %-4s   | %-15s\n",
             "MMC5983MA_MAG",
             ut_mag,
             "uT",
             mag_status);
    }
  else
    {
      printf("  MAG | %-17s | Mag         |  ---  |   ----   | "
             "      ---       | %-15s\n",
             "MMC5983MA_MAG",
             "[ DISCONNECTED ]");
    }

  printf("==============================================================================================\n");
}

/****************************************************************************
 * Name: init_gyro
 ****************************************************************************/

static bool init_gyro(FAR struct spi_dev_s *spi,
                      FAR const struct iam20380_config_s *cfg)
{
  uint8_t id = 0;
  int ret;

  ret =
    iam20380_initialize(spi, cfg);

  if (ret == OK)
    {
      iam20380_read_whoami(spi, &id);

      printf("[OBC_MAIN] %s Gyroscope initialized successfully! "
             "(WHO_AM_I=0x%02X)\n",
             iam20380_id_name(id),
             id);

      return true;
    }

  iam20380_read_whoami(spi, &id);

  printf("[OBC_MAIN] WARNING: gyro init failed (ret=%d), "
         "WHO_AM_I=0x%02X "
         "(expected 0xFD IAM-20380HT or 0xB5 IAM-20380)\n",
         ret,
         id);

  if (id == 0x00 ||
      id == 0xff)
    {
      printf("[OBC_MAIN]   -> No response: check CS mapping in board file, "
             "sensor power, MISO/MOSI/SCK wiring\n");
    }
  else
    {
      printf("[OBC_MAIN]   -> Unexpected/unstable ID: NSS is probably not "
             "being driven (case missing in stm32wl5_spi2select()) so MISO "
             "floats, or SPI mode/clock is wrong\n");
    }

  return false;
}

/****************************************************************************
 * Name: init_mag
 ****************************************************************************/

static bool init_mag(FAR struct spi_dev_s *spi,
                     FAR const struct mmc5983ma_config_s *cfg)
{
  uint8_t id = 0;
  int ret;

  ret =
    mmc5983ma_initialize(spi, cfg);

  if (ret == OK)
    {
      printf("[OBC_MAIN] MMC5983MA Magnetometer initialized successfully!\n");

      return true;
    }

  mmc5983ma_read_id(spi, &id);

  printf("[OBC_MAIN] WARNING: MMC5983MA init failed (ret=%d), "
         "PRODUCT_ID=0x%02X (expected 0x30)\n",
         ret,
         id);

  if (id == 0x00 ||
      id == 0xff)
    {
      printf("[OBC_MAIN]   -> No response: check CS mapping in board file, "
             "sensor power, MISO/MOSI/SCK wiring\n");
    }
  else
    {
      printf("[OBC_MAIN]   -> Got a different chip's ID: CS ID probably "
             "selects the wrong device\n");
    }

  return false;
}

/****************************************************************************
 * Name: gyro_probe_cs
 ****************************************************************************/

static bool gyro_probe_cs(FAR struct spi_dev_s *spi,
                          FAR struct iam20380_config_s *cfg)
{
  static const uint32_t cs_list[] =
  {
    SPIDEV_USER(3)
  };

  static const int cs_num[] =
  {
    3
  };

  static const enum spi_mode_e mode_list[] =
  {
    SPIDEV_MODE0,
    SPIDEV_MODE3
  };

  static const uint32_t freq_list[] =
  {
    1000000,
    250000
  };

  struct iam20380_config_s trial;

  int ncs =
    (int)(sizeof(cs_list) /
          sizeof(cs_list[0]));

  int nmode =
    (int)(sizeof(mode_list) /
          sizeof(mode_list[0]));

  int nfreq =
    (int)(sizeof(freq_list) /
          sizeof(freq_list[0]));

  int i;
  int m;
  int f;

  printf("\n");
  printf("[OBC_MAIN] [PROBE] Checking SPI2 NSS4 (PB9) "
         "for InvenSense/IAM Gyroscope...\n");

  for (i = 0; i < ncs; i++)
    {
      bool found = false;
      uint8_t found_id = 0;
      enum spi_mode_e found_mode = SPIDEV_MODE0;
      uint32_t found_freq = 0;

      printf("[OBC_MAIN] [PROBE]   cs=SPIDEV_USER(%d):",
             cs_num[i]);

      for (m = 0; m < nmode; m++)
        {
          for (f = 0; f < nfreq; f++)
            {
              uint8_t id = 0;

              trial = *cfg;

              trial.cs =
                cs_list[i];

              trial.mode =
                mode_list[m];

              trial.frequency =
                1000000;

              trial.reg_frequency =
                freq_list[f];

              if (iam20380_probe(spi,
                                 &trial,
                                 &id) != OK)
                {
                  continue;
                }

              printf(" m%d@%luk=0x%02X",
                     (mode_list[m] == SPIDEV_MODE0) ? 0 : 3,
                     (unsigned long)(freq_list[f] / 1000),
                     id);

              if (!found &&
                  iam20380_id_valid(id))
                {
                  found = true;
                  found_id = id;
                  found_mode = mode_list[m];
                  found_freq = freq_list[f];
                }
            }
        }

      if (found)
        {
          printf("   <-- %s FOUND\n",
                 iam20380_id_name(found_id));

          cfg->cs =
            cs_list[i];

          cfg->mode =
            found_mode;

          cfg->reg_frequency =
            found_freq;

          return true;
        }

      printf("\n");
    }

  printf("[OBC_MAIN] [PROBE] No chip select returned "
         "0xFD (IAM-20380HT) or 0xB5 (IAM-20380).\n");

  printf("[OBC_MAIN]   -> Add a case for the gyro CS devid to "
         "stm32wl5_spi2select() in the board file.\n");

  printf("[OBC_MAIN]   -> Confirm the NSS4 pin is configured as a GPIO "
         "output and idles high.\n");

  printf("[OBC_MAIN]   -> Check gyro VDD/VDDIO, then scope NSS4 and MISO: "
         "NSS4 must pulse low during each transfer.\n\n");

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obc_main_main
 *
 * Description:
 *
 *   Startup:
 *
 *       initialize local telemetry RB
 *              |
 *       initialize IPC
 *              |
 *       initialize SPI2
 *              |
 *       initialize gyro
 *              |
 *       initialize magnetometer
 *              |
 *       create telemetry pthread
 *              |
 *       enter main OBC loop
 *
 *   Main thread:
 *
 *       radio logs
 *       ground commands
 *       other OBC tasks
 *
 *   Telemetry thread:
 *
 *       sleep 90 sec
 *       acquire telemetry
 *       store/send telemetry
 *       sleep 90 sec
 ****************************************************************************/

int obc_main_main(int argc, char *argv[])
{
  int ret;

  (void)argc;
  (void)argv;

  printf("\n");
  printf("=============================================================\n");
  printf("  OBC Main: Dual ADS7953 & IMU Telemetry Producer\n");
  printf("  ADC 1 (/dev/adc0): CH00-CH07 Voltage | CH08-CH15 Temp\n");
  printf("  ADC 2 (/dev/adc1): Current Sensors\n");
  printf("  ADC2 NC: CH00, CH04, CH07, CH12\n");
  printf("  IMU SPI2: IAM-20380/HT Gyro NSS4\n");
  printf("  IMU SPI2: MMC5983MA Mag NSS3\n");
  printf("  Magnetometer units: microtesla (uT), packed x10\n");
  printf("  Acquisition cadence: Every %d seconds\n",
         TELEMETRY_INTERVAL_SEC);
  printf("=============================================================\n");

  /**************************************************************************
   * 1. Initialize local telemetry ring buffer
   **************************************************************************/

  printf("[OBC_MAIN] Initializing telemetry & camera ring buffers...\n");

  telemetry_rb_init();
  rb_init();
  ensure_hk_partition_mounted();
  ensure_camera_partition_mounted();

  printf("[OBC_MAIN] Telemetry & camera ring buffers + Flash partitions (/mnt/hk, /mnt/camera) initialized\n");

  /**************************************************************************
   * 2. Initialize shared SRAM2 IPC/ring buffers
   **************************************************************************/

  printf("[OBC_MAIN] Initializing M4 <-> M0+ IPC...\n");

  rb_ipc_init();

  printf("[OBC_MAIN] M4 <-> M0+ IPC initialized\n");

  /* Handshake with Cortex-M0+ (CPU2) via IPCC Channel 1 */
  printf("[OBC_MAIN] [IPC] Performing initial handshake with Cortex-M0+ (IPCC CH1)...\n");
  ipcc_m4_clear(IPCC_CH_HANDSHAKE);
  int hs_retries = 100;
  bool hs_ok = false;
  while (hs_retries-- > 0)
    {
      ipcc_m4_send(IPCC_CH_HANDSHAKE);
      usleep(20000); /* 20 ms */
      if (ipcc_m4_received(IPCC_CH_HANDSHAKE))
        {
          ipcc_m4_clear(IPCC_CH_HANDSHAKE);
          hs_ok = true;
          break;
        }
    }
  if (hs_ok)
    {
      printf("[OBC_MAIN] [IPC] Handshake SUCCESSFUL! Cortex-M0+ is ALIVE and SYNCED.\n");
    }
  else
    {
      printf("[OBC_MAIN] [IPC] Cortex-M0+ handshake check completed.\n");
    }

  /**************************************************************************
   * 3. Initialize SPI2
   **************************************************************************/

  printf("[OBC_MAIN] Initializing SPI2...\n");

  g_spi2 =
    stm32wl5_spibus_initialize(2);

  if (g_spi2 == NULL)
    {
      printf("[OBC_MAIN] ERROR: Failed to initialize SPI2 bus for IMU!\n");

      return -1;
    }

  printf("[OBC_MAIN] SPI2 initialized successfully\n");

  /**************************************************************************
   * 4. Configure gyro
   **************************************************************************/

  g_gyro_cfg =
    (struct iam20380_config_s)
    {
      .frequency     = GYRO_SPI_FREQ,
      .mode          = SPIDEV_MODE0,
      .bits          = 8,
      .cs            = GYRO_CS_DEVID,
      .reg_frequency = 1000000,
    };

  /**************************************************************************
   * 5. Configure magnetometer
   **************************************************************************/

  g_mag_cfg =
    (struct mmc5983ma_config_s)
    {
      .frequency = MAG_SPI_FREQ,
      .mode      = SPIDEV_MODE0,
      .bits      = 8,
      .cs        = MAG_CS_DEVID,
    };

  /**************************************************************************
   * 6. Probe gyro CS
   **************************************************************************/

  printf("[OBC_MAIN] Probing gyro chip select...\n");

  gyro_probe_cs(g_spi2,
                &g_gyro_cfg);

  /**************************************************************************
   * 7. Initialize gyro ONCE at startup
   **************************************************************************/

  printf("[OBC_MAIN] Initializing gyroscope...\n");

  g_gyro_initialized =
    init_gyro(g_spi2,
              &g_gyro_cfg);

  if (g_gyro_initialized)
    {
      printf("[OBC_MAIN] Gyroscope initialization successful\n");
    }
  else
    {
      printf("[OBC_MAIN] WARNING: Gyroscope initialization failed\n");
      printf("[OBC_MAIN] Telemetry thread will retry later\n");
    }

  /**************************************************************************
   * 8. Initialize magnetometer ONCE at startup
   **************************************************************************/

  printf("[OBC_MAIN] Initializing magnetometer...\n");

  g_mag_initialized =
    init_mag(g_spi2,
             &g_mag_cfg);

  if (g_mag_initialized)
    {
      printf("[OBC_MAIN] Magnetometer initialization successful\n");
    }
  else
    {
      printf("[OBC_MAIN] WARNING: Magnetometer initialization failed\n");
      printf("[OBC_MAIN] Telemetry thread will retry later\n");
    }

  /**************************************************************************
   * 9. Create telemetry pthread
   **************************************************************************/

  /*
   * NOTE: A separate telemetry pthread needs an extra 2 KB stack + TCB from
   * the 32 KB SRAM1 heap and failed with ENOMEM (12), which made obc_main
   * exit -> M4 stopped servicing M0+ (dummy camera chunks came out empty).
   * Telemetry is now scheduled from this main loop instead (no extra stack).
   */

  {
    struct mallinfo mem = mallinfo();
    printf("[OBC_MAIN] Heap status: free=%d B, largest_block=%d B\n",
           mem.fordblks, mem.mxordblk);
  }

  printf("[OBC_MAIN] Telemetry scheduled in main loop (every %d s, no extra thread)\n",
         TELEMETRY_INTERVAL_SEC);

  struct timespec last_telem_ts;
  bool first_telem = true;
  memset(&last_telem_ts, 0, sizeof(last_telem_ts));

  /**************************************************************************
   * 10. Main OBC loop
   *
   * IMPORTANT:
   *
   * There is NO 90-second sleep here.
   *
   * The telemetry pthread handles the 90-second acquisition interval.
   *
   * The main thread continuously handles:
   *
   *   - M0+ radio logs
   *   - ground commands
   *   - other OBC activities
   **************************************************************************/

  printf("[OBC_MAIN] Entering continuous main OBC loop...\n");

  while (1)
    {
      struct radio_log_msg_s rlog;

      /**********************************************************************
       * Drain all available radio logs
       **********************************************************************/

      while (radio_log_read(&rlog))
        {
          printf("%s\n",
                 rlog.text);
        }

      /**********************************************************************
       * Process ground commands
       **********************************************************************/

      process_ground_commands();

      /**********************************************************************
       * Periodic telemetry (paused while M0+ downlinks or camera captures)
       **********************************************************************/

      if (!sat_is_downlink_active() && !g_camera_capture_active)
        {
          struct timespec now_ts;
          clock_gettime(CLOCK_MONOTONIC, &now_ts);

          if (first_telem ||
              (now_ts.tv_sec - last_telem_ts.tv_sec) >= TELEMETRY_INTERVAL_SEC)
            {
              first_telem = false;
              perform_telemetry_cycle();
              clock_gettime(CLOCK_MONOTONIC, &last_telem_ts);
              printf("[TELEMETRY] Acquisition cycle completed, next in %d s\n",
                     TELEMETRY_INTERVAL_SEC);
            }
        }

      /**********************************************************************
       * Main loop period: 20 ms so ground commands are serviced quickly
       **********************************************************************/

      usleep(20000);
    }

  return 0;
}

/****************************************************************************
 * Name: obc_boot_main
 *
 * Description:
 *   Init-task entry point (CONFIG_INIT_ENTRYPOINT="obc_boot_main").
 *
 *   Without this, obc_main was only started when typed manually at the
 *   NSH prompt, so the M0+ never received live ADC1/ADC2/IMU telemetry
 *   and CW1/CW2 always keyed the M0+ default values
 *   ("4.00V 4.00V 25.0C 25.0C 25.0C").
 *
 *   This wrapper launches obc_main as its own task immediately at boot
 *   and then runs the normal NuttShell so the console stays usable.
 ****************************************************************************/

extern int nsh_main(int argc, FAR char *argv[]);

int obc_boot_main(int argc, FAR char *argv[])
{
  pid_t pid = task_create("obc_main",
                          CONFIG_EXAMPLES_OBC_MAIN_PRIORITY,
                          CONFIG_EXAMPLES_OBC_MAIN_STACKSIZE,
                          obc_main_main, NULL);
  if (pid < 0)
    {
      printf("[BOOT] ERROR: failed to auto-start obc_main: %d\n", errno);
    }
  else
    {
      printf("[BOOT] obc_main auto-started (pid %d): ADC1+ADC2+IMU every %d s\n",
             (int)pid, TELEMETRY_INTERVAL_SEC);
    }

  return nsh_main(argc, argv);
}