/****************************************************************************
 * apps/examples/camera_MSN2/camera_MSN2.c
 *
 * Dual OpenMV (RGB + NIR) Camera Mission Application for NuttX (STM32F427)
 * Mirrors bare-metal CAM_Board_Test logic, upgraded to use:
 *   - UART for handshake ("SNAP" -> ASCII size -> "READY")
 *   - SPI  for image data reception from each OpenMV H7 sensor
 *
 * Hardware Mapping (stm32f427-custom):
 *   Console Debug : USART2 -> /dev/console (PA2 TX, PA3 RX)
 *   RGB Handshake : USART3 -> /dev/ttyS1   (PD8 TX, PD9 RX)
 *   RGB SPI Data  : SPI3   -> bus 3        (PC12 MOSI, PC11 MISO, PC10 SCK, PC9 CS)
 *   NIR Handshake : UART7  -> /dev/ttyS4   (PE8 TX, PE7 RX)
 *   NIR SPI Data  : SPI4   -> bus 4        (PE6 MOSI, PE5 MISO, PE2 SCK, PE3 CS)
 *   CMD RX / Out1 : UART4  -> /dev/ttyS2   (PA0 TX, PA1 RX)
 *   CMD RX / Out2 : USART6 -> /dev/ttyS3   (PC6 TX, PC7 RX)
 *   Camera Power  : PB4    -> driven HIGH in stm32_bringup()
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <poll.h>
#include <time.h>

#include <nuttx/spi/spi.h>

struct spi_dev_s;
extern struct spi_dev_s *stm32_spibus_initialize(int bus);

/* ================================================================== */
/*  Hardware Device Paths & SPI Configuration                         */
/* ================================================================== */

/* STM32 serial numbering with USART2 as /dev/console (/dev/ttyS0):
 *   /dev/ttyS1 = USART3 (RGB Handshake: PD8 TX, PD9 RX)
 *   /dev/ttyS2 = UART4  (CMD RX & Raw JPEG Out 1: PA0 TX, PA1 RX)
 *   /dev/ttyS3 = USART6 (CMD RX & Raw JPEG Out 2: PC6 TX, PC7 RX)
 *   /dev/ttyS4 = UART7  (NIR Handshake: PE8 TX, PE7 RX)
 */
#define RGB_UART_DEV     "/dev/ttyS1"
#define CMD_UART4_DEV    "/dev/ttyS2"
#define CMD_USART6_DEV   "/dev/ttyS3"
#define NIR_UART_DEV     "/dev/ttyS4"

#define RGB_SPI_BUS      3
#define NIR_SPI_BUS      4
#define CAM_SPI_FREQ     1000000u       /* 1 MHz (Mode 0, 8-bit, MSB-first) */
#define CAM_SPI_MODE     SPIDEV_MODE0
#define SPI_XFER_CHUNK   256u

/* ================================================================== */
/*  Protocol & Buffer Constants (matching bare-metal CAM_Board_Test)  */
/* ================================================================== */

#define CAM_CMD_BYTE          0xCAU
#define MAX_IMAGE_SIZE        30000u
#define SPI_SYNC_BYTES        1024u     /* sync zero bytes before JPEG (matches OpenMV) */
#define SPI_EXTRA_PAD         256u      /* extra headroom if idle bytes precede sync */
#define IMG_BUF_TOTAL_SIZE    (MAX_IMAGE_SIZE + SPI_SYNC_BYTES + SPI_EXTRA_PAD)

#define SNAP_MAX_ATTEMPTS     8
#define SNAP_TIMEOUT_MS       1000u
#define SPI_READY_DELAY_US    35000u    /* 35 ms settle after "READY" before CS */
#define INTER_CAM_DELAY_US    200000u   /* 200 ms gap between RGB and NIR */

/* ================================================================== */
/*  Static Buffers (kept off the 2 KB task stack)                     */
/* ================================================================== */

static uint8_t  cam_image_buf[IMG_BUF_TOTAL_SIZE];
static uint8_t  spi_tx_dummy[SPI_XFER_CHUNK];
static uint32_t packet_counter = 0;

/* ================================================================== */
/*  Monotonic Millisecond Timer                                       */
/* ================================================================== */

static uint32_t get_tick_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL +
                    (uint64_t)ts.tv_nsec / 1000000ULL);
}

/* ================================================================== */
/*  UART Helpers (115200 8N1 raw, non-blocking)                       */
/* ================================================================== */

static int uart_open_and_configure(const char *devpath)
{
  struct termios t;
  int fd = open(devpath, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("[CAM_BOARD] ERROR: open(%s) failed: %d (%s)\n",
             devpath, errno, strerror(errno));
      return -1;
    }

  if (tcgetattr(fd, &t) == 0)
    {
      cfsetispeed(&t, B115200);
      cfsetospeed(&t, B115200);
      t.c_cflag     = CS8 | CREAD | CLOCAL;
      t.c_iflag     = 0;
      t.c_oflag     = 0;
      t.c_lflag     = 0;
      t.c_cc[VMIN]  = 0;
      t.c_cc[VTIME] = 0;
      tcsetattr(fd, TCSANOW, &t);
    }

  tcflush(fd, TCIOFLUSH);
  return fd;
}

static void uart_flush_rx(int fd)
{
  uint8_t dump[32];
  if (fd < 0)
    {
      return;
    }

  while (read(fd, dump, sizeof(dump)) > 0)
    {
    }

  tcflush(fd, TCIFLUSH);
}

static int uart_write_all(int fd, const uint8_t *buf, size_t len)
{
  size_t sent = 0;
  uint32_t start_ms = get_tick_ms();

  if (fd < 0 || buf == NULL || len == 0)
    {
      return -1;
    }

  while (sent < len)
    {
      ssize_t n = write(fd, &buf[sent], len - sent);
      if (n > 0)
        {
          sent += (size_t)n;
        }
      else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        {
          if ((uint32_t)(get_tick_ms() - start_ms) > 20000u)
            {
              return -1;
            }
          usleep(500);
        }
      else
        {
          return -1;
        }
    }

  return 0;
}

/* ================================================================== */
/*  SPI Image Reception Helper                                        */
/* ================================================================== */

static void spi_exchange_chunked(struct spi_dev_s *spi,
                                 uint8_t *rx_dst,
                                 uint32_t len)
{
  uint32_t offset = 0;

  while (offset < len)
    {
      uint32_t xfer = len - offset;
      if (xfer > SPI_XFER_CHUNK)
        {
          xfer = SPI_XFER_CHUNK;
        }

      SPI_EXCHANGE(spi, spi_tx_dummy, &rx_dst[offset], xfer);
      offset += xfer;
    }
}

/* ================================================================== */
/*  Fetch_Camera_Image                                                */
/*    1. UART Handshake: "SNAP" -> ASCII decimal payload_size         */
/*    2. UART Ready    : "READY"                                      */
/*    3. SPI Transfer  : Assert CS, clock JPEG bytes, align to FF D8  */
/*    4. Forward JPEG  : Transmit to UART4 & USART6                   */
/* ================================================================== */

static void fetch_camera_image(const char *cam_name,
                               int fd_cam_uart,
                               struct spi_dev_s *spi_dev,
                               int fd_uart4,
                               int fd_usart6)
{
  char     uart_rx_buf[16];
  uint32_t payload_size = 0;
  uint8_t  ch = 0;

  printf("\r\n[%s] Requesting Image (UART Handshake + SPI Data)...\r\n",
         cam_name);

  if (fd_cam_uart < 0 || spi_dev == NULL)
    {
      printf("[%s] Error: UART or SPI handle not ready.\r\n", cam_name);
      return;
    }

  /* 1. HANDSHAKE: Retry "SNAP" up to 8 times (1.0 s each) */
  for (int attempt = 1; attempt <= SNAP_MAX_ATTEMPTS && payload_size == 0; attempt++)
    {
      uart_flush_rx(fd_cam_uart);
      uart_write_all(fd_cam_uart, (const uint8_t *)"SNAP", 4);

      /* 2. HANDSHAKE: Receive ASCII payload size string over camera UART */
      memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
      int idx = 0;
      uint32_t line_start = get_tick_ms();

      while (idx < (int)(sizeof(uart_rx_buf) - 1) &&
             (uint32_t)(get_tick_ms() - line_start) < SNAP_TIMEOUT_MS)
        {
          struct pollfd pfd;
          pfd.fd      = fd_cam_uart;
          pfd.events  = POLLIN;
          pfd.revents = 0;

          int pret = poll(&pfd, 1, 20);
          if (pret > 0 && (pfd.revents & POLLIN))
            {
              while (idx < (int)(sizeof(uart_rx_buf) - 1) &&
                     read(fd_cam_uart, &ch, 1) == 1)
                {
                  if (ch == '\n' || ch == '\r')
                    {
                      if (idx > 0)
                        {
                          goto line_done;
                        }
                    }
                  else if (ch >= '0' && ch <= '9')
                    {
                      uart_rx_buf[idx++] = (char)ch;
                    }
                }
            }
        }

line_done:
      payload_size = (uint32_t)atoi(uart_rx_buf);
      if (payload_size == 0)
        {
          printf("[%s] No size response yet (attempt %d/%d)...\r\n",
                 cam_name, attempt, SNAP_MAX_ATTEMPTS);
        }
    }

  /* Safety check against buffer overflow or timeout */
  if (payload_size == 0 || payload_size >= MAX_IMAGE_SIZE)
    {
      printf("[%s] Error: Invalid size %lu bytes.\r\n",
             cam_name, (unsigned long)payload_size);
      return;
    }

  printf("[%s] Expecting %lu bytes (+ %u sync) via SPI Mode 0. Sending READY over UART...\r\n",
         cam_name, (unsigned long)payload_size, (unsigned)SPI_SYNC_BYTES);

  /* 3. HANDSHAKE: Clear image buffer and send "READY" over camera UART */
  memset(cam_image_buf, 0, payload_size + SPI_SYNC_BYTES + SPI_EXTRA_PAD);
  memset(spi_tx_dummy, 0x00, sizeof(spi_tx_dummy));
  uart_flush_rx(fd_cam_uart);
  uart_write_all(fd_cam_uart, (const uint8_t *)"READY", 5);

  /* Give OpenMV time to finish reading "READY" and enter SPI slave send */
  usleep(SPI_READY_DELAY_US);

  /* 4. SPI DATA TRANSFER (Mode 0): Assert CS, read sync + payload, align on FF D8 */
  SPI_LOCK(spi_dev, true);
  SPI_SETMODE(spi_dev, CAM_SPI_MODE);
  SPI_SETBITS(spi_dev, 8);
  SPI_SETFREQUENCY(spi_dev, CAM_SPI_FREQ);
  SPI_SELECT(spi_dev, 0, true);

  uint32_t total_rx = SPI_SYNC_BYTES + payload_size;
  spi_exchange_chunked(spi_dev, cam_image_buf, total_rx);

  /* Search for JPEG Start-of-Image marker (0xFF 0xD8) */
  int soi_offset = -1;
  for (uint32_t k = 0; k + 1 < total_rx; k++)
    {
      if (cam_image_buf[k] == 0xFF && cam_image_buf[k + 1] == 0xD8)
        {
          soi_offset = (int)k;
          break;
        }
    }

  /* If extra idle bytes preceded the sync zeros, clock in the remaining
   * tail bytes while CS is still asserted so we keep the full payload_size
   * JPEG through FF D9.
   */
  if (soi_offset > (int)SPI_SYNC_BYTES)
    {
      uint32_t extra = (uint32_t)soi_offset - SPI_SYNC_BYTES;
      if (extra > SPI_EXTRA_PAD)
        {
          extra = SPI_EXTRA_PAD;
        }
      spi_exchange_chunked(spi_dev, &cam_image_buf[total_rx], extra);
      total_rx += extra;
    }

  SPI_SELECT(spi_dev, 0, false);
  SPI_LOCK(spi_dev, false);

  /* 5. Output Results & Forward JPEG to UART4 & USART6 */
  printf("[%s] SPI Image Transfer Complete (%lu B read)!\r\n",
         cam_name, (unsigned long)total_rx);

  printf("First 24 bytes: ");
  for (uint32_t k = 0; k < 24u && k < total_rx; k++)
    {
      printf("%02X ", cam_image_buf[k]);
    }
  printf("\r\n");

  printf("Bytes @ sync offset (%u): ", (unsigned)SPI_SYNC_BYTES);
  for (uint32_t k = SPI_SYNC_BYTES;
       k < SPI_SYNC_BYTES + 16u && k < total_rx; k++)
    {
      printf("%02X ", cam_image_buf[k]);
    }
  printf("\r\n");

  if (soi_offset >= 0)
    {
      uint8_t *jpeg_start = &cam_image_buf[soi_offset];
      uint32_t valid_len  = total_rx - (uint32_t)soi_offset;
      if (valid_len > payload_size)
        {
          valid_len = payload_size;
        }

      printf("Valid JPEG Detected at offset %d! End bytes: %02X %02X. "
             "Forwarding %lu B via UART4 & USART6...\r\n",
             soi_offset,
             (valid_len >= 2) ? jpeg_start[valid_len - 2] : 0,
             (valid_len >= 1) ? jpeg_start[valid_len - 1] : 0,
             (unsigned long)valid_len);

      /* TRANSMIT OUT: Clean aligned JPEG payload to UART4 & USART6 */
      uart_write_all(fd_uart4, jpeg_start, valid_len);
      uart_write_all(fd_usart6, jpeg_start, valid_len);

      printf("UART4 & USART6 Transfer Complete.\r\n");
    }
  else
    {
      printf("WARNING: FF D8 not found in SPI payload! Forwarding raw buffer anyway...\r\n");
      uart_write_all(fd_uart4, cam_image_buf, payload_size);
      uart_write_all(fd_usart6, cam_image_buf, payload_size);
    }
}

/* ================================================================== */
/*  Entry Point — Always Waiting for Command (same logic as bare)     */
/* ================================================================== */

int camera_MSN2_main(int argc, char *argv[])
{
  int fd_rgb    = -1;
  int fd_nir    = -1;
  int fd_uart4  = -1;
  int fd_usart6 = -1;
  struct spi_dev_s *spi_rgb = NULL;
  struct spi_dev_s *spi_nir = NULL;

  printf("\r\n================ CAMERA MSN2 START ================\r\n");

  /* Open & configure all UARTs (115200 8N1) */
  fd_rgb    = uart_open_and_configure(RGB_UART_DEV);
  fd_nir    = uart_open_and_configure(NIR_UART_DEV);
  fd_uart4  = uart_open_and_configure(CMD_UART4_DEV);
  fd_usart6 = uart_open_and_configure(CMD_USART6_DEV);

  if (fd_uart4 < 0 && fd_usart6 < 0)
    {
      printf("[CAM_BOARD] FATAL: Neither UART4 (%s) nor USART6 (%s) could be opened!\r\n",
             CMD_UART4_DEV, CMD_USART6_DEV);
      goto cleanup;
    }

  /* Initialize SPI3 (RGB: PC10/11/12, CS=PC9) & SPI4 (NIR: PE2/5/6, CS=PE3) */
  spi_rgb = stm32_spibus_initialize(RGB_SPI_BUS);
  if (spi_rgb == NULL)
    {
      printf("[CAM_BOARD] ERROR: SPI%d (RGB) init failed!\r\n", RGB_SPI_BUS);
    }
  else
    {
      SPI_LOCK(spi_rgb, true);
      SPI_SELECT(spi_rgb, 0, false); /* Keep CS PC9 idle HIGH */
      SPI_LOCK(spi_rgb, false);
      printf("[CAM_BOARD] RGB ready: UART=%s (USART3 PD8/PD9), SPI%d (PC10/11/12, CS=PC9)\r\n",
             RGB_UART_DEV, RGB_SPI_BUS);
    }

  spi_nir = stm32_spibus_initialize(NIR_SPI_BUS);
  if (spi_nir == NULL)
    {
      printf("[CAM_BOARD] ERROR: SPI%d (NIR) init failed!\r\n", NIR_SPI_BUS);
    }
  else
    {
      SPI_LOCK(spi_nir, true);
      SPI_SELECT(spi_nir, 0, false); /* Keep CS PE3 idle HIGH */
      SPI_LOCK(spi_nir, false);
      printf("[CAM_BOARD] NIR ready: UART=%s (UART7 PE8/PE7), SPI%d (PE2/5/6, CS=PE3)\r\n",
             NIR_UART_DEV, NIR_SPI_BUS);
    }

  uart_flush_rx(fd_uart4);
  uart_flush_rx(fd_usart6);

  printf("\r\n[CAM_BOARD] Boot OK. Always waiting for command (0xCA / CAM / SNAP)...\r\n");

  char    ascii_cmd[8];
  uint8_t ascii_idx = 0;
  memset(ascii_cmd, 0, sizeof(ascii_cmd));

  /* Infinite command wait loop — exact same trigger logic as bare-metal */
  while (1)
    {
      struct pollfd pfds[2];
      nfds_t nfds = 0;

      if (fd_uart4 >= 0)
        {
          pfds[nfds].fd      = fd_uart4;
          pfds[nfds].events  = POLLIN;
          pfds[nfds].revents = 0;
          nfds++;
        }
      if (fd_usart6 >= 0)
        {
          pfds[nfds].fd      = fd_usart6;
          pfds[nfds].events  = POLLIN;
          pfds[nfds].revents = 0;
          nfds++;
        }

      int pret = poll(pfds, nfds, 100);
      if (pret <= 0)
        {
          continue;
        }

      uint8_t trigger_capture = 0;
      uint8_t cmd_byte = 0;

      for (nfds_t i = 0; i < nfds && !trigger_capture; i++)
        {
          if (!(pfds[i].revents & POLLIN))
            {
              continue;
            }

          while (read(pfds[i].fd, &cmd_byte, 1) == 1)
            {
              if (cmd_byte == CAM_CMD_BYTE)
                {
                  trigger_capture = 1;
                  ascii_idx = 0;
                  memset(ascii_cmd, 0, sizeof(ascii_cmd));
                  break;
                }

              if (ascii_idx < sizeof(ascii_cmd) - 1u)
                {
                  ascii_cmd[ascii_idx++] = (char)cmd_byte;
                  ascii_cmd[ascii_idx]   = '\0';
                }
              else
                {
                  memmove(ascii_cmd, ascii_cmd + 1, sizeof(ascii_cmd) - 2u);
                  ascii_cmd[sizeof(ascii_cmd) - 2u] = (char)cmd_byte;
                  ascii_cmd[sizeof(ascii_cmd) - 1u] = '\0';
                }

              if (strstr(ascii_cmd, "CAM") != NULL ||
                  strstr(ascii_cmd, "SNAP") != NULL)
                {
                  trigger_capture = 1;
                  ascii_idx = 0;
                  memset(ascii_cmd, 0, sizeof(ascii_cmd));
                  break;
                }
            }
        }

      if (trigger_capture)
        {
          packet_counter++;
          printf("\r\n[CAM_BOARD] Command #%lu received! Capturing RGB + NIR...\r\n",
                 (unsigned long)packet_counter);

          /* Notify ADCS/OBC that command was received */
          uart_write_all(fd_uart4, (const uint8_t *)"CAM_OK\n", 7);
          uart_write_all(fd_usart6, (const uint8_t *)"CAM_OK\n", 7);

          /* 1. Fetch image from RGB Camera (USART3 handshake + SPI3 data) */
          fetch_camera_image("RGB", fd_rgb, spi_rgb, fd_uart4, fd_usart6);

          /* 2. Short gap before NIR image */
          usleep(INTER_CAM_DELAY_US);

          /* 3. Fetch image from NIR Camera (UART7 handshake + SPI4 data) */
          fetch_camera_image("NIR", fd_nir, spi_nir, fd_uart4, fd_usart6);

          /* Flush any duplicate command bytes queued during capture */
          uart_flush_rx(fd_uart4);
          uart_flush_rx(fd_usart6);
          ascii_idx = 0;
          memset(ascii_cmd, 0, sizeof(ascii_cmd));

          printf("\r\n[CAM_BOARD] Capture & transmit complete. "
                 "Returning to command waiting mode...\r\n");
        }
    }

cleanup:
  if (fd_rgb >= 0) close(fd_rgb);
  if (fd_nir >= 0) close(fd_nir);
  if (fd_uart4 >= 0) close(fd_uart4);
  if (fd_usart6 >= 0) close(fd_usart6);
  return -1;
}
