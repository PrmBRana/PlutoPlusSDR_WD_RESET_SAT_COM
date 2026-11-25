using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace OpenBirdsOperation
{
    /// <summary>
    /// Direct TCP link between OpenBirdsOperation and GNU Radio (gmsk4800_gs.grc) for Pluto+ SDR.
    ///   - Port 8001 (TX): 128-bit preamble (0xAA*16) + 32-bit sync (93 0B 51 DE) + 200B G3RUH/NRZ-I/HDLC/AX.25
    ///   - Port 8002 (RX): Continuous G3RUH/NRZ-I/HDLC/AX.25 demodulated bitstream
    ///   - Port 8003 (CW): 200 Hz (5 ms/sample) signed dBFS envelope bytes for live CW Morse decoding
    /// </summary>
    public class PlutoSdrLink : IDisposable
    {
        public const string DisplayPortName = "Pluto+ SDR (GNU Radio 127.0.0.1:8001/8002)";

        private const int RadioFixedPacketLen = 200;
        private const int LeadingFlagsDefault = 20;
        private const int TrailingFlushBytes = 16;
        private const string UplinkDestCall = "9NS2S2";
        private const int UplinkDestSsid = 1;
        private const string UplinkSrcCall = "GROUND";
        private const int UplinkSrcSsid = 0;

        private static readonly byte[] SyncWord = { 0x93, 0x0B, 0x51, 0xDE };

        private readonly string host;
        private readonly int txPort;
        private readonly int rxPort;
        private readonly int cwPort;

        private TcpClient txClient;
        private NetworkStream txStream;
        private TcpClient rxClient;
        private NetworkStream rxStream;
        private TcpClient cwClient;
        private NetworkStream cwStream;
        private Thread rxThread;
        private Thread cwThread;
        private volatile bool running;
        private readonly object txLock = new object();
        private long cwMuteUntilTicks = 0;
        private int lastRssiDb = -90;
        private double lastDopplerTxHz = double.NaN;
        private double lastDopplerRxHz = double.NaN;
        private readonly int xmlRpcPort;

        public event Action<string> OnTextReceived;

        public bool IsConnected => running && txClient != null && txClient.Connected;

        public PlutoSdrLink(string host = "127.0.0.1", int txPort = 8001, int rxPort = 8002, int cwPort = 8003, int xmlRpcPort = 8080)
        {
            this.host = host;
            this.txPort = txPort;
            this.rxPort = rxPort;
            this.cwPort = cwPort;
            this.xmlRpcPort = xmlRpcPort;
        }

        /// <summary>
        /// Push live digital Doppler offsets (Hz) to GNU Radio's XML-RPC Server (glitch-free).
        /// </summary>
        public void SetDopplerOffsets(double txOffsetHz, double rxOffsetHz)
        {
            if (!IsConnected) return;
            if (!double.IsNaN(lastDopplerTxHz) &&
                Math.Abs(txOffsetHz - lastDopplerTxHz) < 5.0 &&
                Math.Abs(rxOffsetHz - lastDopplerRxHz) < 5.0)
            {
                return;
            }
            lastDopplerTxHz = txOffsetHz;
            lastDopplerRxHz = rxOffsetHz;

            ThreadPool.QueueUserWorkItem(_ =>
            {
                SendXmlRpcDouble("set_doppler_tx_hz", txOffsetHz);
                SendXmlRpcDouble("set_doppler_rx_hz", rxOffsetHz);
            });
        }

        private void SendXmlRpcDouble(string methodName, double value)
        {
            try
            {
                string valStr = value.ToString("F2", System.Globalization.CultureInfo.InvariantCulture);
                string body =
                    "<?xml version=\"1.0\"?>" +
                    $"<methodCall><methodName>{methodName}</methodName>" +
                    $"<params><param><value><double>{valStr}</double></value></param></params></methodCall>";
                byte[] bodyBytes = Encoding.UTF8.GetBytes(body);
                string req =
                    $"POST /RPC2 HTTP/1.1\r\nHost: {host}:{xmlRpcPort}\r\n" +
                    $"Content-Type: text/xml\r\nContent-Length: {bodyBytes.Length}\r\nConnection: close\r\n\r\n";
                byte[] hdrBytes = Encoding.ASCII.GetBytes(req);

                using (var client = new TcpClient())
                {
                    var ar = client.BeginConnect(host, xmlRpcPort, null, null);
                    if (!ar.AsyncWaitHandle.WaitOne(TimeSpan.FromMilliseconds(300))) return;
                    client.EndConnect(ar);
                    using (var st = client.GetStream())
                    {
                        st.WriteTimeout = 300;
                        st.Write(hdrBytes, 0, hdrBytes.Length);
                        st.Write(bodyBytes, 0, bodyBytes.Length);
                        st.Flush();
                    }
                }
            }
            catch { }
        }

        public void Connect()
        {
            Disconnect();

            txClient = ConnectSingle(host, txPort, "TX");
            txStream = txClient.GetStream();

            try
            {
                rxClient = ConnectSingle(host, rxPort, "RX");
                rxStream = rxClient.GetStream();
            }
            catch
            {
                Disconnect();
                throw;
            }

            bool cwOk = false;
            try
            {
                cwClient = ConnectSingle(host, cwPort, "CW");
                cwStream = cwClient.GetStream();
                cwOk = true;
            }
            catch
            {
                cwClient = null;
                cwStream = null;
            }

            running = true;
            rxThread = new Thread(RxLoop) { IsBackground = true, Name = "PlutoSdrRx" };
            rxThread.Start();

            if (cwOk)
            {
                cwThread = new Thread(CwRxLoop) { IsBackground = true, Name = "PlutoSdrCwRx" };
                cwThread.Start();
            }

            string cwInfo = cwOk ? $" / :{cwPort} CW" : " (CW :8003 not active - reload gmsk4800_gs.grc)";
            EmitText($"[PLUTO SDR] Connected directly to GNU Radio ({host}:{txPort} TX / :{rxPort} RX{cwInfo})\r\n" +
                     $"[PLUTO SDR] Ready | GMSK 4800 bps + CW Morse | Uplink 435.313 MHz | Downlink 437.375 MHz\r\n");
        }

        private static TcpClient ConnectSingle(string host, int port, string label)
        {
            var client = new TcpClient();
            client.NoDelay = true;
            var ar = client.BeginConnect(host, port, null, null);
            if (!ar.AsyncWaitHandle.WaitOne(TimeSpan.FromSeconds(3)))
            {
                try { client.Close(); } catch { }
                throw new Exception($"Timed out connecting to GNU Radio {label} ({host}:{port}). Make sure gmsk4800_gs.grc is running in GNU Radio.");
            }
            client.EndConnect(ar);
            return client;
        }

        public void Disconnect()
        {
            running = false;
            try { cwStream?.Close(); } catch { }
            try { cwClient?.Close(); } catch { }
            try { rxStream?.Close(); } catch { }
            try { rxClient?.Close(); } catch { }
            try { txStream?.Close(); } catch { }
            try { txClient?.Close(); } catch { }
            cwStream = null;
            cwClient = null;
            rxStream = null;
            rxClient = null;
            txStream = null;
            txClient = null;
        }

        public void Dispose()
        {
            Disconnect();
        }

        public void TransmitPayload(byte[] payload, string label = null)
        {
            if (!IsConnected || txStream == null)
            {
                throw new InvalidOperationException("Pluto+ SDR is not connected to GNU Radio.");
            }

            byte[] rfPacket = EncodeRfPacket(payload);
            lock (txLock)
            {
                txStream.Write(rfPacket, 0, rfPacket.Length);
                txStream.Flush();
            }

            string tag = string.IsNullOrEmpty(label) ? BitConverter.ToString(payload).Replace("-", " ") : label;
            EmitText($">>> [PLUTO TX] {tag} ({payload.Length}B payload -> {rfPacket.Length}B RF sent)\r\n");
        }

        private void EmitText(string text)
        {
            var handler = OnTextReceived;
            if (handler != null && !string.IsNullOrEmpty(text))
            {
                handler(text);
            }
        }

        private void RxLoop()
        {
            Action<string> onSyncOrDiag = (msg) =>
            {
                if (msg.Contains("Sync 93 0B 51 DE"))
                {
                    cwMuteUntilTicks = DateTime.UtcNow.Ticks + TimeSpan.TicksPerMillisecond * 1500;
                }
                EmitText(msg);
            };

            // NRZ-I + G3RUH is polarity-invariant, so a single decoder instance avoids duplicate frames
            var decNorm = new G3ruhAx25Decoder(false, onSyncOrDiag);
            byte[] buf = new byte[1024];

            while (running)
            {
                try
                {
                    if (rxStream == null) break;
                    int n = rxStream.Read(buf, 0, buf.Length);
                    if (n <= 0)
                    {
                        EmitText("[PLUTO SDR] GNU Radio RX socket closed.\r\n");
                        break;
                    }

                    var frames = decNorm.FeedBytes(buf, n);
                    foreach (byte[] ax25Body in frames)
                    {
                        cwMuteUntilTicks = DateTime.UtcNow.Ticks + TimeSpan.TicksPerMillisecond * 1500;
                        string msg = FormatRxFrame(ax25Body, lastRssiDb);
                        if (!string.IsNullOrEmpty(msg))
                        {
                            EmitText(msg);
                        }
                    }
                }
                catch
                {
                    if (running)
                    {
                        EmitText("[PLUTO SDR] RX connection interrupted.\r\n");
                    }
                    break;
                }
            }
        }

        private void CwRxLoop()
        {
            var cw = new CwMorseDecoder(EmitText);
            byte[] buf = new byte[256];
            long sampleMs = 0;

            while (running)
            {
                try
                {
                    if (cwStream == null) break;
                    int n = cwStream.Read(buf, 0, buf.Length);
                    if (n <= 0) break;

                    bool muted = DateTime.UtcNow.Ticks < cwMuteUntilTicks;
                    for (int i = 0; i < n; i++)
                    {
                        int rssiDb = (sbyte)buf[i]; // signed dBFS (-128..0) at 5 ms/sample
                        lastRssiDb = rssiDb;
                        sampleMs += 5;
                        if (!muted)
                        {
                            cw.Feed(rssiDb, sampleMs);
                        }
                    }
                }
                catch
                {
                    break;
                }
            }
        }

        // ---- Live CW Morse Decoder (matches Ground_Station/Main_code/Main_Ground_Station/cw_rx.h) ----

        private class CwMorseDecoder
        {
            private const int UnitMs = 80;         // 15 WPM (dit = 80 ms, dah = 240 ms)
            private const int DahMinMs = 160;      // >= 160 ms -> dah
            private const int TuneMinMs = 1200;    // >= 1200 ms -> tuning carrier
            private const int CharGapMs = 160;     // >= 160 ms -> end of char
            private const int WordGapMs = 400;     // >= 400 ms -> word space
            private const int GlitchMs = 40;       // < 40 ms -> noise spike
            private const int LineIdleMs = 4500;   // flush line after 4.5 s silence
            private const int MinSnrDb = 10;       // carrier threshold above noise floor
            private const int DebounceSamples = 3; // 15 ms edge debounce

            private static readonly Dictionary<string, char> MorseTable = new Dictionary<string, char>
            {
                { ".-", 'A' }, { "-...", 'B' }, { "-.-.", 'C' }, { "-..", 'D' },
                { ".", 'E' }, { "..-.", 'F' }, { "--.", 'G' }, { "....", 'H' },
                { "..", 'I' }, { ".---", 'J' }, { "-.-", 'K' }, { ".-..", 'L' },
                { "--", 'M' }, { "-.", 'N' }, { "---", 'O' }, { ".--.", 'P' },
                { "--.-", 'Q' }, { ".-.", 'R' }, { "...", 'S' }, { "-", 'T' },
                { "..-", 'U' }, { "...-", 'V' }, { ".--", 'W' }, { "-..-", 'X' },
                { "-.--", 'Y' }, { "--..", 'Z' },
                { "-----", '0' }, { ".----", '1' }, { "..---", '2' }, { "...--", '3' },
                { "....-", '4' }, { ".....", '5' }, { "-....", '6' }, { "--...", '7' },
                { "---..", '8' }, { "----.", '9' },
                { "-..-.", '/' }, { "-...-", '=' }, { ".-.-.-", '.' }, { "--..--", ',' }, { "..--..", '?' }
            };

            private readonly Action<string> emit;
            private double noiseFloor = -95.0;
            private bool initialized = false;
            private bool keyDown = false;
            private int debounceCnt = 0;
            private long edgeMs = 0;
            private long lastRssiReportMs = 0;
            private bool charFlushed = true;
            private bool wordFlushed = true;
            private bool signalActive = false;
            private readonly StringBuilder symBuf = new StringBuilder(16);
            private readonly StringBuilder lineBuf = new StringBuilder(128);

            public CwMorseDecoder(Action<string> emit)
            {
                this.emit = emit;
            }

            public void Feed(int rssiDb, long nowMs)
            {
                if (!initialized)
                {
                    noiseFloor = rssiDb;
                    initialized = true;
                    edgeMs = nowMs;
                    lastRssiReportMs = nowMs;
                    return;
                }

                // Slow adaptive noise floor tracking when key is up (and fast drop if floor lowers)
                if (!keyDown)
                {
                    if (rssiDb < noiseFloor)
                        noiseFloor = noiseFloor * 0.90 + rssiDb * 0.10;
                    else if (rssiDb < noiseFloor + MinSnrDb - 2)
                        noiseFloor = noiseFloor * 0.995 + rssiDb * 0.005;
                }

                double threshOn = noiseFloor + MinSnrDb;
                double threshOff = noiseFloor + (MinSnrDb - 3);
                bool rawOn = keyDown ? (rssiDb >= threshOff) : (rssiDb >= threshOn);

                // Periodic RSSI heartbeat every 500 ms so the GUI RSSI readout stays live
                if (nowMs - lastRssiReportMs >= 500)
                {
                    lastRssiReportMs = nowMs;
                    emit($"{{\"type\":\"CWSIG\",\"on\":{(keyDown || signalActive ? 1 : 0)},\"rssi\":{rssiDb}}}\r\n");
                }

                if (rawOn != keyDown)
                {
                    debounceCnt++;
                    if (debounceCnt >= DebounceSamples)
                    {
                        debounceCnt = 0;
                        long dur = nowMs - edgeMs;
                        edgeMs = nowMs;

                        if (keyDown)
                        {
                            // Mark ended (carrier went OFF)
                            keyDown = false;
                            if (dur >= TuneMinMs)
                            {
                                symBuf.Clear();
                                emit($"{{\"type\":\"CWCAR\",\"ms\":{dur},\"rssi\":{rssiDb}}}\r\n");
                            }
                            else if (dur >= GlitchMs)
                            {
                                symBuf.Append(dur >= DahMinMs ? '-' : '.');
                                charFlushed = false;
                                wordFlushed = false;
                                signalActive = true;
                            }
                        }
                        else
                        {
                            // Space ended (carrier went ON)
                            keyDown = true;
                            if (!signalActive)
                            {
                                signalActive = true;
                                emit($"{{\"type\":\"CWSIG\",\"on\":1,\"rssi\":{rssiDb}}}\r\n");
                            }
                        }
                    }
                }
                else
                {
                    debounceCnt = 0;
                }

                // Process gaps while key is UP
                if (!keyDown)
                {
                    long spaceDur = nowMs - edgeMs;

                    if (!charFlushed && spaceDur >= CharGapMs && symBuf.Length > 0)
                    {
                        string pattern = symBuf.ToString();
                        symBuf.Clear();
                        char ch = MorseTable.TryGetValue(pattern, out char decoded) ? decoded : '?';
                        lineBuf.Append(ch);
                        charFlushed = true;
                        emit($"{{\"type\":\"CWCHAR\",\"ch\":\"{ch}\",\"morse\":\"{pattern}\",\"rssi\":{rssiDb}}}\r\n");
                    }

                    if (!wordFlushed && charFlushed && spaceDur >= WordGapMs && lineBuf.Length > 0)
                    {
                        if (lineBuf[lineBuf.Length - 1] != ' ')
                        {
                            lineBuf.Append(' ');
                            emit($"{{\"type\":\"CWCHAR\",\"ch\":\" \",\"morse\":\"\",\"rssi\":{rssiDb}}}\r\n");
                        }
                        wordFlushed = true;
                    }

                    if (spaceDur >= LineIdleMs)
                    {
                        if (lineBuf.Length > 0)
                        {
                            string text = lineBuf.ToString().Trim();
                            lineBuf.Clear();
                            int cs = text.IndexOf("9NS2S2", StringComparison.OrdinalIgnoreCase) >= 0 ? 1 : 0;
                            emit($"{{\"type\":\"CWRX\",\"text\":\"{text}\",\"callsign\":{cs}}}\r\n");
                        }
                        if (signalActive)
                        {
                            signalActive = false;
                            emit($"{{\"type\":\"CWSIG\",\"on\":0,\"rssi\":{rssiDb}}}\r\n");
                        }
                    }
                }
            }
        }

        // ---- AX.25 / CRC-16 / HDLC / NRZ-I / G3RUH ----

        public static ushort Crc16X25(byte[] data, int offset, int length)
        {
            ushort crc = 0xFFFF;
            for (int i = 0; i < length; i++)
            {
                crc ^= data[offset + i];
                for (int bit = 0; bit < 8; bit++)
                {
                    if ((crc & 1) != 0)
                        crc = (ushort)((crc >> 1) ^ 0x8408);
                    else
                        crc >>= 1;
                }
            }
            return (ushort)(~crc);
        }

        private static byte[] EncodeAx25Addr(string callsign, int ssid, bool isLast)
        {
            string call = (callsign ?? "").ToUpperInvariant().PadRight(6, ' ').Substring(0, 6);
            byte[] addr = new byte[7];
            for (int i = 0; i < 6; i++)
            {
                addr[i] = (byte)((call[i] << 1) & 0xFE);
            }
            addr[6] = (byte)(0x60 | ((ssid & 0x0F) << 1) | (isLast ? 0x01 : 0x00));
            return addr;
        }

        public static byte[] BuildAx25UiFrame(string dest, int destSsid, string src, int srcSsid, byte[] payload)
        {
            byte[] destAddr = EncodeAx25Addr(dest, destSsid, false);
            byte[] srcAddr = EncodeAx25Addr(src, srcSsid, true);
            byte[] body = new byte[16 + payload.Length];
            Buffer.BlockCopy(destAddr, 0, body, 0, 7);
            Buffer.BlockCopy(srcAddr, 0, body, 7, 7);
            body[14] = 0x03; // UI Control
            body[15] = 0xF0; // No L3 PID
            Buffer.BlockCopy(payload, 0, body, 16, payload.Length);

            ushort fcs = Crc16X25(body, 0, body.Length);
            byte[] frame = new byte[body.Length + 2];
            Buffer.BlockCopy(body, 0, frame, 0, body.Length);
            frame[body.Length] = (byte)(fcs & 0xFF);
            frame[body.Length + 1] = (byte)((fcs >> 8) & 0xFF);
            return frame;
        }

        public static byte[] EncodeRfPacket(byte[] payload)
        {
            byte[] frame = BuildAx25UiFrame(UplinkDestCall, UplinkDestSsid, UplinkSrcCall, UplinkSrcSsid, payload);
            int cap = RadioFixedPacketLen * 8;
            byte[] hdlcBits = new byte[cap];
            int count = 0;
            byte[] flagBits = { 0, 1, 1, 1, 1, 1, 1, 0 };

            int leading = payload.Length > 100 ? 12 : LeadingFlagsDefault;
            for (int f = 0; f < leading + 1; f++)
            {
                for (int b = 0; b < 8 && count < cap; b++)
                    hdlcBits[count++] = flagBits[b];
            }

            int ones = 0;
            for (int i = 0; i < frame.Length; i++)
            {
                byte v = frame[i];
                for (int b = 0; b < 8; b++)
                {
                    byte bit = (byte)((v >> b) & 1);
                    if (count < cap) hdlcBits[count++] = bit;
                    if (bit == 1)
                    {
                        ones++;
                        if (ones == 5)
                        {
                            if (count < cap) hdlcBits[count++] = 0;
                            ones = 0;
                        }
                    }
                    else
                    {
                        ones = 0;
                    }
                }
            }

            int padIdx = 0;
            while (count < cap)
            {
                hdlcBits[count++] = flagBits[padIdx];
                padIdx = (padIdx + 1) & 7;
            }

            // NRZ-I encode
            byte nrziState = 1;
            byte[] nrziBits = new byte[cap];
            for (int i = 0; i < cap; i++)
            {
                if (hdlcBits[i] == 0) nrziState ^= 1;
                nrziBits[i] = nrziState;
            }

            // G3RUH scramble (1 + x^12 + x^17)
            uint sr = 0x1FFFFu;
            byte[] scrambled = new byte[cap];
            for (int i = 0; i < cap; i++)
            {
                byte fb = (byte)(((sr >> 11) ^ (sr >> 16)) & 1u);
                byte outBit = (byte)(nrziBits[i] ^ fb);
                sr = ((sr << 1) | outBit) & 0x1FFFFu;
                scrambled[i] = outBit;
            }

            // Pack MSB-first + prepend 16B preamble (0xAA) + 4B sync (93 0B 51 DE) + trailing flush bytes
            byte[] rfPkt = new byte[16 + 4 + RadioFixedPacketLen + TrailingFlushBytes];
            for (int i = 0; i < 16; i++) rfPkt[i] = 0xAA;
            Buffer.BlockCopy(SyncWord, 0, rfPkt, 16, 4);

            for (int i = 0; i < RadioFixedPacketLen; i++)
            {
                byte val = 0;
                int baseIdx = i * 8;
                for (int b = 0; b < 8; b++)
                {
                    val = (byte)((val << 1) | scrambled[baseIdx + b]);
                }
                rfPkt[20 + i] = val;
            }
            for (int i = 0; i < TrailingFlushBytes; i++)
            {
                rfPkt[20 + RadioFixedPacketLen + i] = 0xAA;
            }

            return rfPkt;
        }

        private static short ReadI16(byte[] buf, int offset)
        {
            return (short)(buf[offset] | (buf[offset + 1] << 8));
        }

        private static string FormatRxFrame(byte[] ax25Body, int rssiDb)
        {
            if (ax25Body == null || ax25Body.Length < 16) return string.Empty;

            var destSb = new StringBuilder(6);
            for (int i = 0; i < 6; i++) destSb.Append((char)((ax25Body[i] >> 1) & 0x7F));
            int destSsid = (ax25Body[6] >> 1) & 0x0F;

            var srcSb = new StringBuilder(6);
            for (int i = 7; i < 13; i++) srcSb.Append((char)((ax25Body[i] >> 1) & 0x7F));
            int srcSsid = (ax25Body[13] >> 1) & 0x0F;

            int plen = ax25Body.Length - 16;
            byte[] pl = new byte[plen];
            Buffer.BlockCopy(ax25Body, 16, pl, 0, plen);

            var hexSpaced = new StringBuilder(plen * 3);
            var hexCompact = new StringBuilder(plen * 2);
            var txtSb = new StringBuilder(plen);
            for (int i = 0; i < plen; i++)
            {
                byte b = pl[i];
                hexSpaced.AppendFormat("{0:X2} ", b);
                hexCompact.AppendFormat("{0:X2}", b);
                txtSb.Append(b >= 32 && b <= 126 ? (char)b : '.');
            }

            var outSb = new StringBuilder();
            outSb.AppendFormat("[GS RX] *** AX.25 DECODE OK ({0}-{1} -> {2}-{3}, {4} B payload) ***\r\n",
                srcSb.ToString().Trim(), srcSsid, destSb.ToString().Trim(), destSsid, plen);

            // Match gs_app.c payload dispatch (ACK / NACK / HK / CAMERA / FLASH)
            if (plen >= 3 && pl[0] == 0xAC)
            {
                string status = pl[1] == 0x01 ? "ACK COMPLETED (0xAC 0x01)" : "ACK ACCEPTED (0xAC 0x00)";
                outSb.AppendFormat("  HEX : {0}\r\n", hexSpaced.ToString().TrimEnd());
                outSb.AppendFormat("[SATELLITE-ACK] {0} from satellite. Mission=0x{1:X2}\r\n", status, pl[2]);
                outSb.AppendFormat("{{\"type\":\"ACK\",\"msg\":\"{0} mission=0x{1:X2}\"}}\r\n", status, pl[2]);
            }
            else if (plen >= 3 && pl[0] == 0xEE)
            {
                outSb.AppendFormat("  HEX : {0}\r\n", hexSpaced.ToString().TrimEnd());
                if (pl[1] != 0x00 && pl[1] != 0x01)
                {
                    outSb.AppendFormat("[SATELLITE-NACK] NACK/ERROR (0xEE) err=0x{0:X2} mission=0x{1:X2}\r\n", pl[1], pl[2]);
                    outSb.AppendFormat("{{\"type\":\"NACK\",\"msg\":\"NACK (0xEE) err=0x{0:X2} mission=0x{1:X2}\"}}\r\n", pl[1], pl[2]);
                }
                else
                {
                    outSb.AppendFormat("[SATELLITE-ACK] ACK COMPLETED (0xEE 0x{0:X2}) mission=0x{1:X2}\r\n", pl[1], pl[2]);
                    outSb.AppendFormat("{{\"type\":\"ACK\",\"msg\":\"ACK Completed (0xEE) mission=0x{0:X2}\"}}\r\n", pl[2]);
                }
            }
            else if (plen >= 3 && pl[0] == 0xFF)
            {
                outSb.AppendFormat("  HEX : {0}\r\n", hexSpaced.ToString().TrimEnd());
                outSb.AppendFormat("[SATELLITE-NACK] NACK (0xFF) err=0x{0:X2}\r\n", pl[1]);
                outSb.AppendFormat("{{\"type\":\"NACK\",\"msg\":\"NACK (0xFF) err=0x{0:X2}\"}}\r\n", pl[1]);
            }
            else if (plen == 128 && ((pl[32] == 0x55 && pl[33] == 0xAA) || (pl[32] == 0xAA && pl[33] == 0x55)))
            {
                bool f1ok = (pl[32] == 0x55 && pl[33] == 0xAA);
                bool f2ok = (pl[70] == 0x66 && pl[71] == 0xBB);
                bool enok = (pl[126] == 0xCC && pl[127] == 0xAA);
                uint seq = (uint)(pl[72] | (pl[73] << 8) | (pl[74] << 16) | (pl[75] << 24));
                outSb.AppendLine(">>> [GS RX] HOUSEKEEPING TELEMETRY (128 BYTES) <<<");
                outSb.AppendFormat(
                    "{{\"type\":\"HK\",\"vbat\":{0},\"vsol\":{1},\"vbus\":{2}," +
                    "\"sp5\":{3},\"sp4\":{4},\"sp3\":{5},\"sp1\":{6},\"sp2\":{7}," +
                    "\"t_ant\":{8},\"t_bat\":{9},\"t_bpb\":{10}," +
                    "\"i_unreg\":{11},\"i_3v3\":{12},\"i_5v\":{13},\"i_bat\":{14},\"i_bus\":{15}," +
                    "\"f1\":{16},\"f2\":{17}," +
                    "\"gx\":{18},\"gy\":{19},\"gz\":{20},\"mx\":{21},\"my\":{22},\"mz\":{23}," +
                    "\"seq\":{24},\"crc\":{25},\"rssi\":{26},\"hex\":\"{27}\"}}\r\n",
                    ReadI16(pl, 0), ReadI16(pl, 2), ReadI16(pl, 4),
                    ReadI16(pl, 6), ReadI16(pl, 8), ReadI16(pl, 10), ReadI16(pl, 12), ReadI16(pl, 14),
                    ReadI16(pl, 16), ReadI16(pl, 18), ReadI16(pl, 20),
                    ReadI16(pl, 34), ReadI16(pl, 36), ReadI16(pl, 38), ReadI16(pl, 40), ReadI16(pl, 52),
                    ReadI16(pl, 54), ReadI16(pl, 56),
                    ReadI16(pl, 58), ReadI16(pl, 60), ReadI16(pl, 62), ReadI16(pl, 64), ReadI16(pl, 66), ReadI16(pl, 68),
                    seq, (f1ok && f2ok && enok) ? 1 : 0, rssiDb, hexCompact);
            }
            else if (plen >= 14 && ((pl[0] == 0xFE && pl[1] == 0xCA) || (pl[0] == 0xCA && pl[1] == 0xFE) ||
                                    (pl[0] == 0xDA && pl[1] == 0xDB) || (pl[0] == 0xDB && pl[1] == 0xDA) ||
                                    (pl[0] == 0xF0 && pl[1] == 0xFD) || (pl[0] == 0xFD && pl[1] == 0xF0)))
            {
                bool isCam = (pl[0] == 0xFE && pl[1] == 0xCA) || (pl[0] == 0xCA && pl[1] == 0xFE);
                const int hdrLen = 14;
                int pktIdx = pl[2] | (pl[3] << 8);
                int total = pl[4] | (pl[5] << 8);
                uint offset = (uint)(pl[6] | (pl[7] << 8) | (pl[8] << 16) | (pl[9] << 24));
                int dataLen = pl[10] | (pl[11] << 8);
                byte status = pl[12];
                int maxData = plen - hdrLen;
                if (dataLen <= 0 || dataLen > maxData) dataLen = maxData;

                var dataCompact = new StringBuilder(dataLen * 2);
                var dataSpaced = new StringBuilder(dataLen * 3);
                for (int i = 0; i < dataLen; i++)
                {
                    byte b = pl[hdrLen + i];
                    dataCompact.AppendFormat("{0:X2}", b);
                    dataSpaced.AppendFormat("{0:X2} ", b);
                }

                outSb.AppendFormat("[GS {0}] Pkt {1}/{2}  off=0x{3:X4}  len={4}  status={5}\r\n",
                    isCam ? "CAM" : "FLASH", pktIdx + 1, total, offset, dataLen, status);
                if (isCam)
                {
                    outSb.AppendFormat("{{\"type\":\"CAMERA\",\"pkt\":{0},\"total\":{1},\"offset\":{2},\"len\":{3},\"hex\":\"{4}\"}}\r\n",
                        pktIdx + 1, total, offset, dataLen, dataCompact);
                }
                else
                {
                    outSb.AppendFormat("{{\"type\":\"FLASH\",\"pkt\":{0},\"total\":{1},\"addr\":{2},\"len\":{3},\"status\":{4}}}\r\n",
                        pktIdx + 1, total, offset, dataLen, status);
                    outSb.AppendFormat("[FLASH-HEX] {0}\r\n", dataSpaced.ToString().TrimEnd());
                }
            }
            else
            {
                outSb.AppendFormat("  HEX : {0}\r\n", hexSpaced.ToString().TrimEnd());
                outSb.AppendFormat("  TEXT: {0}\r\n", txtSb);
            }

            return outSb.ToString();
        }

        private class G3ruhAx25Decoder
        {
            private const uint SyncWordU32 = 0x930B51DEu;
            private readonly bool invert;
            private readonly Action<string> diagEmit;
            private uint sr = 0x1FFFFu;
            private uint syncSr = 0u;
            private long lastSyncLogTicks = 0;
            private byte prevNrzi = 0;
            private bool inFrame = false;
            private int ones = 0;
            private int bitBuf = 0;
            private int bitCnt = 0;
            private readonly List<byte> frameBytes = new List<byte>(256);

            public G3ruhAx25Decoder(bool invert, Action<string> diagEmit = null)
            {
                this.invert = invert;
                this.diagEmit = diagEmit;
            }

            private static int PopCount32(uint x)
            {
                int c = 0;
                while (x != 0) { x &= x - 1; c++; }
                return c;
            }

            public List<byte[]> FeedBytes(byte[] raw, int length)
            {
                var frames = new List<byte[]>();
                for (int i = 0; i < length; i++)
                {
                    byte b = raw[i];
                    for (int shift = 7; shift >= 0; shift--)
                    {
                        byte rxBit = (byte)(((b >> shift) & 1) ^ (invert ? 1 : 0));

                        // Sync word diagnostic (93 0B 51 DE with <= 2 bit errors)
                        syncSr = (syncSr << 1) | rxBit;
                        int syncErrs = PopCount32(syncSr ^ SyncWordU32);
                        if (syncErrs <= 2 && diagEmit != null)
                        {
                            long nowTicks = DateTime.UtcNow.Ticks;
                            if (nowTicks - lastSyncLogTicks > TimeSpan.TicksPerMillisecond * 400)
                            {
                                lastSyncLogTicks = nowTicks;
                                diagEmit($"[PLUTO RX-DIAG] Sync 93 0B 51 DE detected (inv={(invert ? 1 : 0)}, bitErrs={syncErrs})\r\n");
                            }
                        }

                        byte fb = (byte)(((sr >> 11) ^ (sr >> 16)) & 1u);
                        byte descrambled = (byte)(rxBit ^ fb);
                        sr = ((sr << 1) | rxBit) & 0x1FFFFu;

                        byte hdlcBit = (byte)(descrambled == prevNrzi ? 1 : 0);
                        prevNrzi = descrambled;

                        if (hdlcBit == 0)
                        {
                            if (ones == 6)
                            {
                                if (inFrame && bitCnt == 7 && frameBytes.Count >= 18)
                                {
                                    byte[] candidate = frameBytes.ToArray();
                                    ushort calc = Crc16X25(candidate, 0, candidate.Length - 2);
                                    ushort rxFcsLe = (ushort)(candidate[candidate.Length - 2] | (candidate[candidate.Length - 1] << 8));
                                    ushort rxFcsBe = (ushort)((candidate[candidate.Length - 2] << 8) | candidate[candidate.Length - 1]);
                                    if (calc == rxFcsLe || calc == rxFcsBe)
                                    {
                                        byte[] body = new byte[candidate.Length - 2];
                                        Buffer.BlockCopy(candidate, 0, body, 0, body.Length);
                                        frames.Add(body);
                                    }
                                    else if (diagEmit != null && candidate[14] == 0x03 && candidate[15] == 0xF0)
                                    {
                                        diagEmit($"[PLUTO RX-DIAG] AX.25 UI frame ({candidate.Length}B) CRC mismatch: rx=0x{rxFcsLe:X4} calc=0x{calc:X4}\r\n");
                                    }
                                }
                                inFrame = true;
                                ones = 0;
                                bitBuf = 0;
                                bitCnt = 0;
                                frameBytes.Clear();
                                continue;
                            }
                            else if (ones == 5)
                            {
                                ones = 0;
                                continue;
                            }
                            ones = 0;
                        }
                        else
                        {
                            ones++;
                            if (ones >= 7)
                            {
                                inFrame = false;
                                frameBytes.Clear();
                                bitBuf = 0;
                                bitCnt = 0;
                                continue;
                            }
                        }

                        if (!inFrame) continue;

                        bitBuf |= (hdlcBit << bitCnt);
                        bitCnt++;
                        if (bitCnt == 8)
                        {
                            frameBytes.Add((byte)bitBuf);
                            bitBuf = 0;
                            bitCnt = 0;
                            if (frameBytes.Count > 330)
                            {
                                inFrame = false;
                                frameBytes.Clear();
                            }
                        }
                    }
                }
                return frames;
            }
        }
    }
}
