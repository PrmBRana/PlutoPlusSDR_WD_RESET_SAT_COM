/*
 * ADCS.c  —  NuttX ADCS task
 *
 * UART7 = OBC interface (handshake, command, image data out, NACK)
 * UART8 = CAM interface (command out, JPEG data in)
 *
 * Sequence:
 *   1. Wait for handshake "MSN" from OBC on UART7
 *   2. Echo "MSN" back to OBC on UART7
 *   3. Wait for command from OBC on UART7:
 *      - 0xCA  : Real camera capture -> send 53 04 CC 5E BD... to CAM on UART8,
 *                receive two JPEGs (FF D8 ... FF D9) with 30 s timeout,
 *                stream via ring buffer to OBC on UART7 (or NACK on timeout/null).
 *      - "CAM" : Dummy camera capture -> stream g_dummy_camera_image
 *                (camera_dummy_image.h, FF D8 ... FF D9) via ring buffer
 *                to OBC on UART7.
 *      - NULL / invalid / timeout -> reject and send NACK (0xEE) to OBC.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <termios.h>

#include "ring_buffer.h"
#include "camera_dummy_image.h"

/* ---- Device configuration ----
 * On stm32f427-custom when USART1 and USART2 (console) are both enabled:
 *   /dev/ttyS0 = USART2 (Console)
 *   /dev/ttyS1 = USART1 (PA9/PA10)
 *   /dev/ttyS2 = UART7  (PE7 RX / PE8 TX) -> OBC interface
 *   /dev/ttyS3 = UART8  (PE0 RX / PE1 TX) -> CAM interface
 */
#ifndef UART7_DEV
#  if defined(CONFIG_STM32_USART1) && defined(CONFIG_STM32_USART2)
#    define UART7_DEV     "/dev/ttyS2"   /* UART7 (PE7 RX, PE8 TX): OBC interface */
#  else
#    define UART7_DEV     "/dev/ttyS1"   /* UART7: OBC interface */
#  endif
#endif
#ifndef UART8_DEV
#  if defined(CONFIG_STM32_USART1) && defined(CONFIG_STM32_USART2)
#    define UART8_DEV     "/dev/ttyS3"   /* UART8 (PE0 RX, PE1 TX): CAM interface */
#  else
#    define UART8_DEV     "/dev/ttyS2"   /* UART8: CAM interface */
#  endif
#endif

#define UART7_BAUD        B115200
#define UART8_BAUD        B115200

#define CMD_FROM_OBC      0xCA           /* Real camera command */
#define CMD_DUMMY_OBC     0xD0           /* Internal code returned when "CAM" dummy command received */
#define CMD_INVALID_OBC   0x00           /* Null / rejected command */
#define NACK_BYTE         0xEE           /* change to your protocol's NACK */
#define IMAGE_COUNT       2
#define IMAGE_TIMEOUT_MS  30000
#define CMD_TIMEOUT_MS    10000          /* 10 s timeout for command after handshake */
#define HS_MAX_LEN        64

static const uint8_t g_cmd_cam[13] =
{
  0x53, 0x04, 0xCC, 0x5E, 0xBD,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */

static uint32_t now_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static int uart_configure(int fd, speed_t baud)
{
  struct termios t;

  if (tcgetattr(fd, &t) < 0)
    {
      return -1;
    }

  cfmakeraw(&t);
  cfsetspeed(&t, baud);
  t.c_cflag |= (CLOCAL | CREAD);
  t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  t.c_cc[VMIN]  = 0;
  t.c_cc[VTIME] = 0;

  return tcsetattr(fd, TCSANOW, &t);
}

static void uart_flush(int fd)
{
  uint8_t tmp[64];
  while (read(fd, tmp, sizeof(tmp)) > 0)
    {
    }
}

static int uart_open(const char *dev, speed_t baud)
{
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("[ADCS] open %s failed: %s\n", dev, strerror(errno));
      return -1;
    }

  if (uart_configure(fd, baud) < 0)
    {
      printf("[ADCS] configure %s failed: %s\n", dev, strerror(errno));
      close(fd);
      return -1;
    }

  uart_flush(fd);
  return fd;
}

/* Write all bytes, tolerating EAGAIN on non-blocking fd */
static int uart_write_all(int fd, const uint8_t *buf, size_t len)
{
  size_t sent = 0;

  while (sent < len)
    {
      ssize_t n = write(fd, buf + sent, len - sent);
      if (n > 0)
        {
          sent += (size_t)n;
        }
      else if (n < 0 && errno != EAGAIN && errno != EINTR)
        {
          return -1;
        }
      else
        {
          usleep(500);
        }
    }

  return 0;
}

static void send_nack(int fd_obc)
{
  uint8_t nack = NACK_BYTE;
  uart_write_all(fd_obc, &nack, 1);
  printf("[ADCS] NACK (0x%02X) sent to OBC\n", NACK_BYTE);
}

/* ------------------------------------------------------------------ */
/*  Step 1: wait for "MSN" handshake from OBC on UART7                */
/* ------------------------------------------------------------------ */
static int wait_handshake(int fd, uint8_t *buf, int maxlen)
{
  int msn_idx = 0;
  uint8_t b = 0;

  (void)maxlen;

  /* Wait specifically for "MSN" handshake sequence */
  while (msn_idx < 3)
    {
      ssize_t n = read(fd, &b, 1);
      if (n == 1)
        {
          if (msn_idx == 0 && b == 'M')
            {
              msn_idx = 1;
            }
          else if (msn_idx == 1 && b == 'S')
            {
              msn_idx = 2;
            }
          else if (msn_idx == 2 && b == 'N')
            {
              msn_idx = 3;
            }
          else
            {
              msn_idx = (b == 'M') ? 1 : 0;
            }
        }
      else
        {
          usleep(5000);
        }
    }

  buf[0] = 'M';
  buf[1] = 'S';
  buf[2] = 'N';
  return 3;
}

/* ------------------------------------------------------------------ */
/*  Step 3: wait for command 0xCA (real) or "CAM" (dummy) from OBC    */
/*          Rejects if command read is null, invalid, or times out.   */
/* ------------------------------------------------------------------ */
static uint8_t wait_for_command(int fd_obc, uint8_t expect)
{
  uint8_t b;
  int cam_idx = 0;
  int msn_idx = 0;
  uint32_t t0 = now_ms();

  for (; ; )
    {
      if ((now_ms() - t0) >= CMD_TIMEOUT_MS)
        {
          printf("[ADCS] Command read timeout (null command) -> Rejecting\n");
          send_nack(fd_obc);
          return CMD_INVALID_OBC;
        }

      ssize_t n = read(fd_obc, &b, 1);
      if (n == 1)
        {
          printf("[ADCS] OBC rx: 0x%02X\n", b);

          /* Real camera command: 0xCA */
          if (b == expect)
            {
              return CMD_FROM_OBC;
            }

          /* Dummy camera command: "CAM" (3 bytes 'C', 'A', 'M') */
          if (cam_idx == 0 && b == 'C')
            {
              cam_idx = 1;
              continue;
            }
          else if (cam_idx == 1 && b == 'A')
            {
              cam_idx = 2;
              continue;
            }
          else if (cam_idx == 2 && b == 'M')
            {
              return CMD_DUMMY_OBC;
            }
          else
            {
              cam_idx = (b == 'C') ? 1 : 0;
            }

          /* If OBC retried sending "MSN" handshake, re-echo "MSN" instead of NACK */
          if (msn_idx == 0 && b == 'M')
            {
              msn_idx = 1;
              continue;
            }
          else if (msn_idx == 1 && b == 'S')
            {
              msn_idx = 2;
              continue;
            }
          else if (msn_idx == 2 && b == 'N')
            {
              const uint8_t msn_echo[3] = { 'M', 'S', 'N' };
              uart_write_all(fd_obc, msn_echo, 3);
              printf("[ADCS] Re-echoed 'MSN' handshake to OBC\n");
              msn_idx = 0;
              t0 = now_ms();
              continue;
            }
          else
            {
              msn_idx = (b == 'M') ? 1 : 0;
            }

          /* Reject NULL (0x00) or any unrecognized command byte */
          printf("[ADCS] Invalid or NULL command 0x%02X -> Rejecting\n", b);
          send_nack(fd_obc);
          return CMD_INVALID_OBC;
        }
      else
        {
          usleep(2000);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Ring buffer -> OBC (UART7)                                         */
/* ------------------------------------------------------------------ */
static void drain_rb_to_obc(int fd_obc)
{
  struct image_chunk_s c;

  while (rb_read(&c) == 0)
    {
      uart_write_all(fd_obc, c.data, c.len);
      usleep(2000);
    }
}

struct rx_state_s
{
  struct image_chunk_s chunk;
  uint32_t frame_id;     /* image index 0..IMAGE_COUNT-1      */
  uint8_t  in_image;     /* 1 = between FF D8 and FF D9       */
  uint8_t  prev;
  int      images_done;
  uint32_t total_bytes;
};

static void flush_chunk(struct rx_state_s *s, int fd_obc, uint8_t is_last)
{
  s->chunk.is_last  = is_last;
  s->chunk.is_meta  = 0;
  s->chunk.frame_id = s->frame_id;

  if (rb_write(&s->chunk) < 0)
    {
      /* Full: drain then retry */
      drain_rb_to_obc(fd_obc);
      rb_write(&s->chunk);
    }

  drain_rb_to_obc(fd_obc);

  s->chunk.len = 0;
}

static void add_byte(struct rx_state_s *s, int fd_obc, uint8_t b)
{
  if (s->chunk.len >= CHUNK_SIZE)
    {
      flush_chunk(s, fd_obc, 0);
    }

  s->chunk.data[s->chunk.len++] = b;
  s->total_bytes++;
}

/* Feed CAM bytes through the FFD8 ... FFD9 state machine */
static void process_bytes(struct rx_state_s *s, int fd_obc,
                          const uint8_t *buf, size_t n)
{
  for (size_t i = 0; i < n && s->images_done < IMAGE_COUNT; i++)
    {
      uint8_t b = buf[i];

      if (!s->in_image)
        {
          if (s->prev == 0xFF && b == 0xD8)
            {
              s->in_image  = 1;
              s->chunk.len = 0;
              add_byte(s, fd_obc, 0xFF);
              add_byte(s, fd_obc, 0xD8);
              s->prev = 0;   /* avoid reuse of D8 as prefix */
              continue;
            }
        }
      else
        {
          add_byte(s, fd_obc, b);

          if (s->prev == 0xFF && b == 0xD9)
            {
              flush_chunk(s, fd_obc, 1);
              printf("[ADCS] image %d complete (total %lu bytes)\n",
                     s->images_done + 1, (unsigned long)s->total_bytes);
              s->images_done++;
              s->frame_id++;
              s->in_image = 0;
              s->prev     = 0;
              continue;
            }
        }

      s->prev = b;
    }
}

/* ------------------------------------------------------------------ */
/*  Main                                                               */
/* ------------------------------------------------------------------ */
int ADCS_main(int argc, char *argv[])
{
  int     fd_obc = -1, fd_cam = -1, ret = -1;
  uint8_t hs[HS_MAX_LEN];
  int     hs_len;
  uint8_t cmd_received;

  (void)argc;
  (void)argv;

  printf("\n================ ADCS START ================\n");
  printf("[ADCS] UART7 (OBC, PE7 RX / PE8 TX) -> %s\n", UART7_DEV);
  printf("[ADCS] UART8 (CAM, PE0 RX / PE1 TX) -> %s\n", UART8_DEV);
  rb_init();

  fd_obc = uart_open(UART7_DEV, UART7_BAUD);   /* OBC */
  fd_cam = uart_open(UART8_DEV, UART8_BAUD);   /* CAM */

  if (fd_obc < 0)
    {
      goto out;
    }

  for (; ; )
    {
      rb_init();
      uart_flush(fd_obc);

      /* ---- 1. Wait "MSN" handshake from OBC ---- */
      printf("\n========== Handshake waiting ('MSN' on UART7 %s) ==========\n",
             UART7_DEV);
      hs_len = wait_handshake(fd_obc, hs, sizeof(hs));
      printf("[ADCS] Handshake received (%d bytes):", hs_len);
      for (int i = 0; i < hs_len; i++)
        {
          printf(" %02X", hs[i]);
        }
      printf("\n");

      /* ---- 2. Echo the same "MSN" bytes back to OBC ---- */
      uart_flush(fd_obc);
      if (uart_write_all(fd_obc, hs, hs_len) < 0)
        {
          printf("[ADCS] UART7 handshake echo failed\n");
          continue;
        }
      printf("[ADCS] Handshake 'MSN' echoed to OBC\n");

      /* ---- 3. Wait for command 0xCA (Real) or "CAM" (Dummy) from OBC ---- */
      printf("\n========== Command waiting (0x%02X or 'CAM' on UART7) ==========\n",
             CMD_FROM_OBC);
      cmd_received = wait_for_command(fd_obc, CMD_FROM_OBC);

      if (cmd_received == CMD_INVALID_OBC)
        {
          printf("[ADCS] Command rejected (null or invalid)\n");
          continue;
        }

      if (cmd_received == CMD_DUMMY_OBC)
        {
          printf("[ADCS] Command 'CAM' (Dummy) received -> Sending dummy image x%d (%u bytes each) to OBC on UART7\n",
                 IMAGE_COUNT, (unsigned)DUMMY_CAM_RAW_IMAGE_SIZE);

          struct rx_state_s st;
          memset(&st, 0, sizeof(st));

          for (int img = 0; img < IMAGE_COUNT; img++)
            {
              process_bytes(&st, fd_obc, g_dummy_camera_image, DUMMY_CAM_RAW_IMAGE_SIZE);
              drain_rb_to_obc(fd_obc);
              if (img + 1 < IMAGE_COUNT)
                {
                  usleep(50000);
                }
            }

          printf("[ADCS] %d dummy images (%lu bytes total) sent to OBC via ring buffer!\n",
                 st.images_done, (unsigned long)st.total_bytes);
          ret = 0;
          continue;
        }

      printf("[ADCS] Command 0x%02X received and validated\n", CMD_FROM_OBC);

      if (fd_cam < 0)
        {
          printf("[ADCS] UART8 (CAM) not open, sending NACK\n");
          send_nack(fd_obc);
          continue;
        }

      /* ---- 4. Send command to CAM on UART8 ---- */
      uart_flush(fd_cam);
      if (uart_write_all(fd_cam, g_cmd_cam, sizeof(g_cmd_cam)) < 0)
        {
          printf("[ADCS] UART8 cmd write failed\n");
          send_nack(fd_obc);
          continue;
        }
      printf("[ADCS] Command sent to CAM\n");

      /* ---- 5. Receive two images from CAM, 30 s timeout ---- */
      printf("\n========== Image data waiting (30 s) ==========\n");

      struct rx_state_s st;
      memset(&st, 0, sizeof(st));

      uint32_t t0 = now_ms();
      uint8_t  rx[128];
      bool     timed_out = false;

      while (st.images_done < IMAGE_COUNT)
        {
          if ((now_ms() - t0) >= IMAGE_TIMEOUT_MS)
            {
              printf("[ADCS] Timeout: %d/%d images received\n",
                     st.images_done, IMAGE_COUNT);
              drain_rb_to_obc(fd_obc);   /* send whatever is already queued */
              send_nack(fd_obc);         /* 6b. tell OBC it failed */
              timed_out = true;
              break;
            }

          ssize_t n = read(fd_cam, rx, sizeof(rx));
          if (n > 0)
            {
              /* 6. CAM bytes -> ring buffer -> OBC */
              process_bytes(&st, fd_obc, rx, (size_t)n);
            }
          else
            {
              usleep(1000);
            }
        }

      if (!timed_out)
        {
          drain_rb_to_obc(fd_obc);
          printf("[ADCS] Both images received and forwarded to OBC\n");
          ret = 0;
        }
    }

out:
  if (fd_obc >= 0) close(fd_obc);
  if (fd_cam >= 0) close(fd_cam);
  return ret;
}