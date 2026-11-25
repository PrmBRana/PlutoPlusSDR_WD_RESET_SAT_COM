#!/usr/bin/env python3
"""
Pluto+ SDR Ground Station Bridge for OpenBirdsOperation (Desktop_GS_SOFTWARE)

Bridges:
  OpenBirdsOperation (Serial /dev/ttyUSB9 via socat)
      <--> [This Bridge: AX.25 + CRC-16 + HDLC + NRZ-I + G3RUH + Preamble/Sync]
      <--> GNU Radio gmsk4800_gs.grc (TCP TX :8001, TCP RX :8002)
      <--> Pluto+ SDR

Matches Ground_Station/Mission/config.h and Mission/protocol.c:
  - GMSK 4800 bps, fdev = +/-1200 Hz, BT = 0.5
  - Preamble: 128 bits (16 x 0xAA)
  - Sync Word: 93 0B 51 DE
  - Fixed 200-byte G3RUH-scrambled NRZ-I HDLC AX.25 UI frame
  - Callsigns: Dest=9NS2S2-1 / Src=GROUND-0 (for uplink)
"""

import argparse
import os
import select
import socket
import sys
import termios
import threading
import time
import tty

# ---- RF & Protocol Constants (from Mission/config.h) ----
PREAMBLE_BYTES = bytes([0xAA] * 16)
SYNC_WORD_BYTES = bytes([0x93, 0x0B, 0x51, 0xDE])
RADIO_FIXED_PACKET_LEN = 200
LEADING_FLAGS = 16

UPLINK_DEST_CALL = "9NS2S2"
UPLINK_DEST_SSID = 1
UPLINK_SRC_CALL = "GROUND"
UPLINK_SRC_SSID = 0

# Built-in ASCII command shortcuts sent by OpenBirdsOperation (operationMain.cs)
SHORTCUT_COMMANDS = {
    "DC": bytes.fromhex("53 04 CC 5E BE 00 D0 00 00 00 00 00 45"),
    "CAMON": bytes.fromhex("53 04 CC 5E BC 00 00 00 00 00 00 00 00"),
    "CAM": bytes.fromhex("53 04 CC 5E BD 00 00 00 00 00 00 00 00"),
    "HK1": bytes.fromhex("53 01 1D D1 F2 00 F2 00 00 00 00 00 01"),
    "HK2": bytes.fromhex("53 01 1D D2 F2 00 F2 00 00 00 00 00 01"),
    "PING": bytes.fromhex("53 01 1D D1 F5 00 00 00 00 00 00 00 01"),
}


# ---- AX.25 / CRC-16 / HDLC / NRZ-I / G3RUH ----

def crc16_x25(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return (~crc) & 0xFFFF


def encode_ax25_addr(callsign: str, ssid: int, is_last: bool) -> bytes:
    call = callsign.upper().ljust(6)[:6]
    addr = bytearray((ord(c) << 1) & 0xFE for c in call)
    ssid_byte = 0x60 | ((ssid & 0x0F) << 1) | (0x01 if is_last else 0x00)
    addr.append(ssid_byte)
    return bytes(addr)


def build_ax25_ui_frame(dest: str, dest_ssid: int, src: str, src_ssid: int, payload: bytes) -> bytes:
    body = (
        encode_ax25_addr(dest, dest_ssid, is_last=False)
        + encode_ax25_addr(src, src_ssid, is_last=True)
        + bytes([0x03, 0xF0])
        + payload
    )
    fcs = crc16_x25(body)
    return body + bytes([fcs & 0xFF, (fcs >> 8) & 0xFF])


def encode_rf_packet(payload: bytes) -> bytes:
    """Build 220-byte RF packet: 16B preamble + 4B sync + 200B G3RUH/NRZ-I/HDLC AX.25."""
    frame = build_ax25_ui_frame(
        UPLINK_DEST_CALL, UPLINK_DEST_SSID, UPLINK_SRC_CALL, UPLINK_SRC_SSID, payload
    )
    cap = RADIO_FIXED_PACKET_LEN * 8
    flag_bits = [0, 1, 1, 1, 1, 1, 1, 0]
    hdlc_bits = []

    leading = 12 if len(payload) > 100 else LEADING_FLAGS
    for _ in range(leading + 1):  # leading + opening flag
        hdlc_bits.extend(flag_bits)

    ones = 0
    for byte in frame:
        for b in range(8):
            bit = (byte >> b) & 1
            hdlc_bits.append(bit)
            if bit == 1:
                ones += 1
                if ones == 5:
                    hdlc_bits.append(0)
                    ones = 0
            else:
                ones = 0

    # Closing flags + flag padding to fixed 200 bytes
    while len(hdlc_bits) < cap:
        for fb in flag_bits:
            if len(hdlc_bits) < cap:
                hdlc_bits.append(fb)
    hdlc_bits = hdlc_bits[:cap]

    # NRZ-I encode
    nrzi_state = 1
    nrzi_bits = []
    for bit in hdlc_bits:
        if bit == 0:
            nrzi_state ^= 1
        nrzi_bits.append(nrzi_state)

    # G3RUH scramble (1 + x^12 + x^17, taps 11 & 16, init 0x1FFFF)
    sr = 0x1FFFF
    scrambled_bits = []
    for bit in nrzi_bits:
        fb = ((sr >> 11) ^ (sr >> 16)) & 1
        out_bit = bit ^ fb
        sr = ((sr << 1) | out_bit) & 0x1FFFF
        scrambled_bits.append(out_bit)

    # Pack MSB-first (matches SX126x FIFO & GNU Radio GFSK Mod do_unpack=True)
    pkt = bytearray(RADIO_FIXED_PACKET_LEN)
    for i in range(RADIO_FIXED_PACKET_LEN):
        val = 0
        base = i * 8
        for b in range(8):
            val = (val << 1) | scrambled_bits[base + b]
        pkt[i] = val

    return PREAMBLE_BYTES + SYNC_WORD_BYTES + bytes(pkt)


class G3RUHAX25Decoder:
    """Continuous streaming G3RUH -> NRZ-I -> HDLC -> AX.25 CRC-16 decoder."""

    def __init__(self, invert: bool = False):
        self.invert = invert
        self.sr = 0x1FFFF
        self.prev_nrzi = 0
        self.flag_sr = 0
        self.in_frame = False
        self.ones = 0
        self.bit_buf = 0
        self.bit_cnt = 0
        self.frame_bytes = bytearray()

    def feed_bytes(self, raw: bytes):
        frames = []
        for byte in raw:
            for shift in range(7, -1, -1):  # MSB-first (matches Pack K Bits)
                rx_bit = ((byte >> shift) & 1) ^ (1 if self.invert else 0)

                # G3RUH descramble
                fb = ((self.sr >> 11) ^ (self.sr >> 16)) & 1
                descrambled = rx_bit ^ fb
                self.sr = ((self.sr << 1) | rx_bit) & 0x1FFFF

                # NRZ-I decode (same -> 1, transition -> 0)
                hdlc_bit = 1 if descrambled == self.prev_nrzi else 0
                self.prev_nrzi = descrambled

                # HDLC flag / abort / bit de-stuffing state machine
                if hdlc_bit == 0:
                    if self.ones == 6:
                        # Flag 0x7E (01111110): previous 7 bits (0 + six 1s) are in bit_buf
                        if self.in_frame and self.bit_cnt == 7 and len(self.frame_bytes) >= 18:
                            f = bytes(self.frame_bytes)
                            if self._verify_ax25(f):
                                frames.append(f[:-2])  # strip 2-byte FCS
                        self.in_frame = True
                        self.ones = 0
                        self.bit_buf = 0
                        self.bit_cnt = 0
                        self.frame_bytes.clear()
                        continue
                    elif self.ones == 5:
                        # Stuffed zero bit: discard
                        self.ones = 0
                        continue
                    self.ones = 0
                else:
                    self.ones += 1
                    if self.ones >= 7:
                        # 7+ consecutive 1s = HDLC abort
                        self.in_frame = False
                        self.frame_bytes.clear()
                        self.bit_buf = 0
                        self.bit_cnt = 0
                        continue

                if not self.in_frame:
                    continue

                # Collect LSB-first bytes
                self.bit_buf |= hdlc_bit << self.bit_cnt
                self.bit_cnt += 1
                if self.bit_cnt == 8:
                    self.frame_bytes.append(self.bit_buf)
                    self.bit_buf = 0
                    self.bit_cnt = 0
                    if len(self.frame_bytes) > 330:
                        self.in_frame = False
                        self.frame_bytes.clear()
        return frames

    @staticmethod
    def _verify_ax25(frame_with_fcs: bytes) -> bool:
        body = frame_with_fcs[:-2]
        rx_fcs = frame_with_fcs[-2] | (frame_with_fcs[-1] << 8)
        return crc16_x25(body) == rx_fcs


def format_rx_frame_for_gs(ax25_body: bytes) -> str:
    """Format decoded AX.25 UI frame as readable ASCII text for OpenBirdsOperation."""
    if len(ax25_body) < 16:
        return ""
    dest = "".join(chr((b >> 1) & 0x7F) for b in ax25_body[0:6]).strip()
    dest_ssid = (ax25_body[6] >> 1) & 0x0F
    src = "".join(chr((b >> 1) & 0x7F) for b in ax25_body[7:13]).strip()
    src_ssid = (ax25_body[13] >> 1) & 0x0F
    payload = ax25_body[16:]
    hex_str = " ".join(f"{b:02X}" for b in payload)
    ascii_str = "".join(chr(b) if 32 <= b <= 126 else "." for b in payload)
    return (
        f"[PLUTO RX] {src}-{src_ssid} -> {dest}-{dest_ssid} ({len(payload)} B)\r\n"
        f"  HEX : {hex_str}\r\n"
        f"  TEXT: {ascii_str}\r\n"
    )


def parse_gs_command(line: str):
    """Parse ASCII line from OpenBirdsOperation into raw payload bytes."""
    cmd = line.strip().upper()
    if not cmd:
        return None, None
    if cmd == "STATUS":
        return "STATUS", None
    if cmd in SHORTCUT_COMMANDS:
        return cmd, SHORTCUT_COMMANDS[cmd]
    if cmd.startswith("HEX "):
        hex_part = cmd[4:].strip()
        try:
            return cmd, bytes.fromhex(hex_part)
        except ValueError:
            return f"INVALID_HEX:{cmd}", None
    # Try raw hex bytes directly
    try:
        return cmd, bytes.fromhex(cmd)
    except ValueError:
        return f"UNKNOWN:{cmd}", None


def open_raw_serial(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def connect_tcp_retry(host: str, port: int, label: str):
    while True:
        try:
            s = socket.create_connection((host, port), timeout=5.0)
            s.settimeout(None)
            print(f"[BRIDGE] Connected to GNU Radio {label} ({host}:{port})")
            return s
        except OSError:
            print(f"[BRIDGE] Waiting for GNU Radio {label} ({host}:{port})...")
            time.sleep(2.0)


def run_self_test():
    test_payload = SHORTCUT_COMMANDS["PING"]
    rf_pkt = encode_rf_packet(test_payload)
    assert len(rf_pkt) == 220, f"Expected 220B, got {len(rf_pkt)}"
    dec = G3RUHAX25Decoder(invert=False)
    frames = dec.feed_bytes(rf_pkt)
    assert len(frames) == 1, f"Expected 1 decoded frame, got {len(frames)}"
    assert frames[0][16:] == test_payload, "Decoded payload mismatch!"
    print("[SELF-TEST] PASS: TX encode -> RX decode round-trip OK!")
    print(format_rx_frame_for_gs(frames[0]))


def main():
    parser = argparse.ArgumentParser(description="Pluto+ GNU Radio <-> OpenBirdsOperation Bridge")
    parser.add_argument("--pty", default="/tmp/gs_bridge_pty", help="Virtual serial PTY path from socat")
    parser.add_argument("--gr-host", default="127.0.0.1", help="GNU Radio TCP host")
    parser.add_argument("--tx-port", type=int, default=8001, help="GNU Radio TX Socket PDU port")
    parser.add_argument("--rx-port", type=int, default=8002, help="GNU Radio RX Socket PDU port")
    parser.add_argument("--self-test", action="store_true", help="Run TX->RX AX.25/G3RUH self-test and exit")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return

    print(f"[BRIDGE] Opening virtual serial port: {args.pty}")
    serial_fd = open_raw_serial(args.pty)
    serial_lock = threading.Lock()

    def write_serial(text: str):
        with serial_lock:
            try:
                os.write(serial_fd, text.encode("ascii", errors="replace"))
            except OSError as e:
                print(f"[BRIDGE] Serial write error: {e}")

    # Thread 1: GNU Radio RX (:8002) -> Decode -> OpenBirdsOperation Serial
    def rx_worker():
        dec_norm = G3RUHAX25Decoder(invert=False)
        dec_inv = G3RUHAX25Decoder(invert=True)
        while True:
            sock = connect_tcp_retry(args.gr_host, args.rx_port, "RX")
            try:
                while True:
                    chunk = sock.recv(1024)
                    if not chunk:
                        break
                    for frame in dec_norm.feed_bytes(chunk) + dec_inv.feed_bytes(chunk):
                        msg = format_rx_frame_for_gs(frame)
                        if msg:
                            print(msg.strip())
                            write_serial(msg)
            except OSError as e:
                print(f"[BRIDGE] RX socket error: {e}")
            finally:
                sock.close()

    threading.Thread(target=rx_worker, daemon=True).start()

    # Main loop: OpenBirdsOperation Serial -> Encode -> GNU Radio TX (:8001)
    tx_sock = connect_tcp_retry(args.gr_host, args.tx_port, "TX")
    write_serial("Pluto+ SDR Ground Station Bridge Ready (TX 435.313 MHz, RX 437.375 MHz).\r\n")

    line_buf = bytearray()
    while True:
        r, _, _ = select.select([serial_fd], [], [], 0.2)
        if not r:
            continue
        try:
            data = os.read(serial_fd, 256)
        except BlockingIOError:
            continue
        if not data:
            time.sleep(0.05)
            continue

        for b in data:
            if b in (10, 13):  # \n or \r
                if not line_buf:
                    continue
                line = line_buf.decode("ascii", errors="ignore")
                line_buf.clear()
                tag, payload = parse_gs_command(line)
                if tag == "STATUS":
                    write_serial("[PLUTO GS] Bridge ONLINE | TX=435.313MHz RX=437.375MHz | GMSK 4800\r\n")
                elif payload is not None:
                    rf_pkt = encode_rf_packet(payload)
                    try:
                        tx_sock.sendall(rf_pkt)
                        msg = f">>> [PLUTO TX] {tag} ({len(payload)}B payload -> {len(rf_pkt)}B RF sent)\r\n"
                        print(msg.strip())
                        write_serial(msg)
                    except OSError:
                        tx_sock.close()
                        tx_sock = connect_tcp_retry(args.gr_host, args.tx_port, "TX")
                        tx_sock.sendall(rf_pkt)
                else:
                    print(f"[BRIDGE] Ignored serial line: {line!r}")
            else:
                line_buf.append(b)


if __name__ == "__main__":
    main()
