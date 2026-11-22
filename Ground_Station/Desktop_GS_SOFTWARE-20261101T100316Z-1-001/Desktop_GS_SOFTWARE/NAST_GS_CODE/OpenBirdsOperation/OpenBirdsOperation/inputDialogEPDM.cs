using OfficeOpenXml;
using OfficeOpenXml.Style;
using System;
using System.Data;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace OpenBirdsOperation
{
    public partial class inputDialogEPDM : Form
    {
        public string savingEPDMRootPath;
        public string EPDMRootPath;

        public DateTime dtAppStart = DateTime.UtcNow;

        private readonly System.Text.StringBuilder _liveHexStream = new System.Text.StringBuilder();
        private bool _liveHeaderFound = false;
        private bool _liveFooterFound = false;
        private int _liveFrameCount = 0;
        private int _lastPkt = 0;
        private int _totalPkts = 0;
        private readonly System.Collections.Generic.HashSet<int> _rxPkts = new System.Collections.Generic.HashSet<int>();
        private RichTextBox rTBox_epdm_live_decoded;
        private Label lbl_epdm_live_status;

        public inputDialogEPDM()
        {
            InitializeComponent();
            InitLiveDecodeUi();
        }

        private void InitLiveDecodeUi()
        {
            this.ClientSize = new System.Drawing.Size(Math.Max(this.ClientSize.Width, 680), 460);
            rTBox_epdm_raw_data.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            rTBox_epdm_raw_data.Width = this.ClientSize.Width - 18;
            rTBox_epdm_raw_data.Height = 110;

            lbl_epdm_live_status = new Label
            {
                Text = "Live Decoded EPDM Magnetometer Frames (waiting for 'EPDM' header 45 50 44 4D ... FF D9):",
                Location = new System.Drawing.Point(9, rTBox_epdm_raw_data.Bottom + 8),
                Size = new System.Drawing.Size(this.ClientSize.Width - 18, 20),
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
                Font = new System.Drawing.Font("Segoe UI", 9F, System.Drawing.FontStyle.Bold)
            };

            rTBox_epdm_live_decoded = new RichTextBox
            {
                Location = new System.Drawing.Point(9, lbl_epdm_live_status.Bottom + 4),
                Size = new System.Drawing.Size(this.ClientSize.Width - 18, this.ClientSize.Height - (lbl_epdm_live_status.Bottom + 56)),
                Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right,
                ReadOnly = true,
                Font = new System.Drawing.Font("Consolas", 9.5F, System.Drawing.FontStyle.Regular),
                BackColor = System.Drawing.Color.FromArgb(18, 24, 34),
                ForeColor = System.Drawing.Color.FromArgb(134, 239, 172)
            };

            btn_verify_EPDM.Location = new System.Drawing.Point(this.ClientSize.Width - 210, this.ClientSize.Height - 40);
            btn_verify_EPDM.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            button_cancel_epdm.Location = new System.Drawing.Point(this.ClientSize.Width - 105, this.ClientSize.Height - 40);
            button_cancel_epdm.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;

            this.Controls.Add(lbl_epdm_live_status);
            this.Controls.Add(rTBox_epdm_live_decoded);
        }

        private static string ExtractCleanHex(string line)
        {
            if (string.IsNullOrWhiteSpace(line)) return string.Empty;
            string s = line;
            int idx = s.IndexOf("[FLASH-HEX]", StringComparison.OrdinalIgnoreCase);
            if (idx >= 0)
            {
                s = s.Substring(idx + "[FLASH-HEX]".Length);
            }
            return Regex.Replace(s, @"[^0-9a-fA-F]", "").ToUpperInvariant();
        }

        private void UpdateLiveStatusLabel()
        {
            string pktInfo = (_totalPkts > 0)
                ? $"Pkt #{_lastPkt}/{_totalPkts} (Rx: {_rxPkts.Count}/{_totalPkts}) | "
                : "";
            if (_liveFooterFound)
                lbl_epdm_live_status.Text = $"{pktInfo}EPDM Complete ({_liveFrameCount} samples, Footer FF D9)";
            else if (_liveHeaderFound)
                lbl_epdm_live_status.Text = $"{pktInfo}Live Decoded Samples: {_liveFrameCount} (waiting for FF D9...)";
            else
                lbl_epdm_live_status.Text = $"{pktInfo}Waiting for EPDM header (45 50 44 4D)...";
        }

        public void AppendLiveEpdmLine(string line)
        {
            if (this.IsDisposed) return;
            if (this.InvokeRequired)
            {
                try { this.BeginInvoke(new Action<string>(AppendLiveEpdmLine), line); } catch { }
                return;
            }

            if (line.IndexOf("\"type\":\"FLASH\"", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                Match mp = Regex.Match(line, "\"pkt\"\\s*:\\s*(\\d+)");
                Match mt = Regex.Match(line, "\"total\"\\s*:\\s*(\\d+)");
                if (mp.Success && mt.Success &&
                    int.TryParse(mp.Groups[1].Value, out int pn) &&
                    int.TryParse(mt.Groups[1].Value, out int tn))
                {
                    if (pn == 1 || tn != _totalPkts) _rxPkts.Clear();
                    _lastPkt = pn;
                    _totalPkts = tn;
                    _rxPkts.Add(pn);
                    UpdateLiveStatusLabel();
                }
                return;
            }

            string hex = ExtractCleanHex(line);
            if (string.IsNullOrEmpty(hex)) return;

            if (rTBox_epdm_raw_data.TextLength > 0)
            {
                rTBox_epdm_raw_data.AppendText(Environment.NewLine);
            }
            rTBox_epdm_raw_data.AppendText(InsertSpacesAfterEveryTwoBytes(hex));
            rTBox_epdm_raw_data.SelectionStart = rTBox_epdm_raw_data.TextLength;
            rTBox_epdm_raw_data.ScrollToCaret();

            _liveHexStream.Append(hex);
            DecodeBufferedEpdmFrames();
        }

        private void DecodeBufferedEpdmFrames()
        {
            // Stream format: "EPDM" (4550444D) + N x 48-byte samples (96 hex chars each) + "FFD9" footer
            while (!_liveFooterFound)
            {
                string buf = _liveHexStream.ToString();

                if (!_liveHeaderFound)
                {
                    int hdrIdx = buf.IndexOf("4550444D", StringComparison.Ordinal);
                    if (hdrIdx < 0)
                    {
                        if (_liveHexStream.Length > 6)
                        {
                            _liveHexStream.Remove(0, _liveHexStream.Length - 6);
                        }
                        break;
                    }

                    _liveHexStream.Remove(0, hdrIdx + 8);
                    _liveHeaderFound = true;
                    string hdrMsg = "=== EPDM HEADER RECEIVED: 45 50 44 4D ('EPDM') — Live Decoding 48-Byte Samples ===\r\n";
                    rTBox_epdm_live_decoded.AppendText(hdrMsg);
                    UpdateLiveStatusLabel();
                    Console.Write(hdrMsg);
                    continue;
                }

                // Check for end-of-stream footer FF D9 at current sample boundary
                if (buf.Length >= 4 && buf.StartsWith("FFD9", StringComparison.Ordinal))
                {
                    _liveHexStream.Remove(0, 4);
                    _liveFooterFound = true;
                    string ftrMsg = $"=== EPDM FOOTER RECEIVED: FF D9 | Complete! Total: {_liveFrameCount} sample(s) ===\r\n";
                    rTBox_epdm_live_decoded.AppendText(ftrMsg);
                    rTBox_epdm_live_decoded.SelectionStart = rTBox_epdm_live_decoded.TextLength;
                    rTBox_epdm_live_decoded.ScrollToCaret();
                    UpdateLiveStatusLabel();
                    Console.Write(ftrMsg);
                    break;
                }

                // Need 48 bytes (96 hex chars) for the next 4-sensor RM3100 sample
                if (buf.Length < 96)
                {
                    break;
                }

                string payload = buf.Substring(0, 96);
                _liveHexStream.Remove(0, 96);
                _liveFrameCount++;

                var sb = new System.Text.StringBuilder();
                sb.AppendLine($"--- EPDM Sample #{_liveFrameCount} (48 B) ---");
                for (int s = 0; s < 4; s++)
                {
                    int off = s * 24;
                    double x = ReadInt32BE(payload, off) / 100.0;
                    double y = ReadInt32BE(payload, off + 8) / 100.0;
                    double z = ReadInt32BE(payload, off + 16) / 100.0;
                    double mag = Math.Sqrt(x * x + y * y + z * z);
                    sb.AppendLine($"  RM3100 S{s + 1}: X={x,8:F2} uT | Y={y,8:F2} uT | Z={z,8:F2} uT | |B|={mag,7:F2} uT");
                }

                string decoded = sb.ToString();
                rTBox_epdm_live_decoded.AppendText(decoded);
                rTBox_epdm_live_decoded.SelectionStart = rTBox_epdm_live_decoded.TextLength;
                rTBox_epdm_live_decoded.ScrollToCaret();
                UpdateLiveStatusLabel();
                Console.Write(decoded);
            }
        }

        private void button_cancel_epdm_Click(object sender, EventArgs e)
        {
            rTBox_epdm_raw_data.Text = "";
            this.Close();
        }

        static string InsertSpacesAfterEveryTwoBytes(string input)
        {
            var sb = new System.Text.StringBuilder();

            // Split the input into separate lines, preserving the newline characters
            var lines = input.Split(new[] { '\n' }, StringSplitOptions.None);

            foreach (var line in lines)
            {
                // Remove all spaces and other whitespace characters from each line
                var cleanedLine = new string(line.Where(c => !char.IsWhiteSpace(c)).ToArray());

                // Insert spaces after every two bytes in the cleaned line
                for (int i = 0; i < cleanedLine.Length; i += 2)
                {
                    // Append two characters followed by a space
                    sb.Append(cleanedLine.Substring(i, Math.Min(2, cleanedLine.Length - i)) + " ");
                }

                // Add a newline to separate each formatted line
                sb.AppendLine();
            }

            // Return the formatted result with spaces between every two bytes and newlines preserved
            return sb.ToString().TrimEnd();  // Remove any trailing newline that might remain
        }

        private static int ReadInt32BE(string hex, int charOffset)
        {
            uint val = Convert.ToUInt32(hex.Substring(charOffset, 8), 16);
            return unchecked((int)val);
        }

        private void btn_verify_EPDM_Click(object sender, EventArgs e)
        {
            try
            {
                string rawEPDMSRx = string.Join("",
                    rTBox_epdm_raw_data.Lines.Select(l => ExtractCleanHex(l)));

                savingEPDMRootPath = "../Storage/EPDM_DETAILS";
                Properties.Settings.Default.savePathEPDM = savingEPDMRootPath;
                Properties.Settings.Default.Save();

                EPDMRootPath = Properties.Settings.Default.savePathEPDM;
                savingEPDMRootPath = EPDMRootPath;

                if (!Directory.Exists(savingEPDMRootPath))
                {
                    Directory.CreateDirectory(savingEPDMRootPath);
                }

                string time = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss");
                string absolutePath = Path.Combine(savingEPDMRootPath, time + "_EPDM_Data_Verification.xlsx");

                // Stream format: single "EPDM" (4550444D) header + N x 48B (96 hex chars) samples + single FFD9 footer
                var samplePayloads = new System.Collections.Generic.List<string>();
                int streamStart = rawEPDMSRx.IndexOf("4550444D", StringComparison.Ordinal);
                if (streamStart >= 0)
                {
                    int pos = streamStart + 8;
                    int streamEnd = rawEPDMSRx.LastIndexOf("FFD9", StringComparison.Ordinal);
                    if (streamEnd < pos) streamEnd = rawEPDMSRx.Length;
                    while (pos + 96 <= streamEnd)
                    {
                        samplePayloads.Add(rawEPDMSRx.Substring(pos, 96));
                        pos += 96;
                    }
                }
                if (samplePayloads.Count > 0)
                {
                    ExcelPackage.LicenseContext = OfficeOpenXml.LicenseContext.NonCommercial;
                    var summary = new System.Text.StringBuilder();
                    using (var package = new ExcelPackage(new FileInfo(absolutePath)))
                    {
                        var ws = package.Workbook.Worksheets.Add("EPDM_Mag_Data");
                        string[] headers = {
                            "Frame #", "Header",
                            "S1_X (uT)", "S1_Y (uT)", "S1_Z (uT)",
                            "S2_X (uT)", "S2_Y (uT)", "S2_Z (uT)",
                            "S3_X (uT)", "S3_Y (uT)", "S3_Z (uT)",
                            "S4_X (uT)", "S4_Y (uT)", "S4_Z (uT)",
                            "Footer", "Raw Hex"
                        };
                        for (int c = 0; c < headers.Length; c++)
                        {
                            ws.Cells[1, c + 1].Value = headers[c];
                        }
                        ws.Row(1).Style.Font.Bold = true;

                        int row = 2;
                        int frameNum = 1;
                        foreach (string payload in samplePayloads)
                        {
                            ws.Cells[row, 1].Value = frameNum;
                            ws.Cells[row, 2].Value = "EPDM (4550444D)";

                            if (frameNum <= 3)
                            {
                                summary.AppendLine($"Frame #{frameNum}:");
                            }

                            for (int s = 0; s < 4; s++)
                            {
                                int baseOff = s * 24;
                                double x = ReadInt32BE(payload, baseOff) / 100.0;
                                double y = ReadInt32BE(payload, baseOff + 8) / 100.0;
                                double z = ReadInt32BE(payload, baseOff + 16) / 100.0;
                                ws.Cells[row, 3 + s * 3].Value = x;
                                ws.Cells[row, 4 + s * 3].Value = y;
                                ws.Cells[row, 5 + s * 3].Value = z;

                                if (frameNum <= 3)
                                {
                                    double mag = Math.Sqrt(x * x + y * y + z * z);
                                    summary.AppendLine($"  S{s + 1}: X={x:F2} uT, Y={y:F2} uT, Z={z:F2} uT (|B|={mag:F2} uT)");
                                }
                            }

                            ws.Cells[row, 15].Value = "FFD9";
                            ws.Cells[row, 16].Value = InsertSpacesAfterEveryTwoBytes(payload);
                            row++;
                            frameNum++;
                        }

                        ws.Cells[1, 1, row - 1, headers.Length].AutoFitColumns();
                        package.Save();
                    }

                    MessageBox.Show(
                        $"Decoded {samplePayloads.Count} EPDM sample(s).\n\n{summary}\nSaved to: {absolutePath}",
                        "EPDM Decoded Data",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                    this.Close();
                    return;
                }

                string sequenceToExclude = "404040404040E0729C64A692406103F0";
                string result = RemoveAndSeparateSequences(rawEPDMSRx, sequenceToExclude);
                Console.WriteLine(result);
                saveToExcel(result, absolutePath, this);
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                       $"EPDM Operation Invalid \n {ex.Message}",
                       "Please input valid raw data",
                       MessageBoxButtons.OK,
                       MessageBoxIcon.Warning
                );
            }
        }

        static string RemoveAndSeparateSequences(string input, string sequenceToExclude)
        {
            bool firstOccurrence = true;

            // Replace occurrences of the sequence with a delimiter (space) after the first occurrence
            string result = Regex.Replace(input, sequenceToExclude, match =>
            {
                if (firstOccurrence)
                {
                    firstOccurrence = false;
                    return "";  // Return the sequence without a space for the first occurrence
                }
                else
                {
                    return "\n";  // Add space after subsequent occurrences
                }
            });

            return result.Trim(); // Remove any leading/trailing spaces that might remain
        }

        static void saveToExcel(string result, string absolutePath, Form form)
        {
            // Set the license context to NonCommercial or Commercial based on your use case
            ExcelPackage.LicenseContext = OfficeOpenXml.LicenseContext.NonCommercial;  // or LicenseContext.Commercial for commercial use

            if (!File.Exists(absolutePath))
            {
                // Create a new Excel file if it doesn't exist
                using (var package = new ExcelPackage(new FileInfo(absolutePath)))
                {
                    // Add a worksheet to the package
                    var worksheet = package.Workbook.Worksheets.Add("Data");

                    // Add headers to the first row
                    worksheet.Cells[1, 1].Value = "Packet Type";     // Column A: Packet Type
                    worksheet.Cells[1, 2].Value = "Packet Number";   // Column B: Packet Number
                    worksheet.Cells[1, 3].Value = "Count Packet";   // Column C: count packet
                    worksheet.Cells[1, 4].Value = "Raw Data";        // Column D: Raw Data

                    worksheet.Columns[1].AutoFit();
                    worksheet.Columns[2].AutoFit();
                    worksheet.Columns[3].AutoFit();

                    // Make the first row bold
                    worksheet.Row(1).Style.Font.Bold = true;

                    // Split the result into parts based on spaces (subsequent sequences)
                    var sequences = result.Split(new[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);

                    int rowIndex = 2; // Starting row in Excel (row 1)
                    foreach (var sequence in sequences)
                    {
                        if (sequence.Length >= 2)
                        {
                            // Write the first byte to column A
                            worksheet.Cells[rowIndex, 1].Value = sequence.Substring(0, 2);

                            // Write the second byte to column B
                            worksheet.Cells[rowIndex, 2].Value = sequence.Substring(2, 2);

                            worksheet.Cells[rowIndex, 3].Formula = $"=IF(ISBLANK(B{rowIndex}),\"\", IFERROR(HEX2DEC(B{rowIndex}), \"ERROR\"))";

                            // Write the remaining bytes to column C (Raw Data) with spaces after every two bytes
                            string rawData = sequence.Length > 4 ? sequence.Substring(4) : "";
                            worksheet.Cells[rowIndex, 4].Value = InsertSpacesAfterEveryTwoBytes(rawData);

                            rowIndex++; // Move to the next row
                        }
                    }

                    int lastRow = worksheet.Dimension.End.Row;
                    int lastColumn = worksheet.Dimension.End.Column;

                    // Define the dynamic range based on data
                    var range = worksheet.Cells[1, 1, lastRow, lastColumn];

                    // Apply borders dynamically to the range containing data
                    range.Style.Border.Top.Style = ExcelBorderStyle.Thin;
                    range.Style.Border.Left.Style = ExcelBorderStyle.Thin;
                    range.Style.Border.Right.Style = ExcelBorderStyle.Thin;
                    range.Style.Border.Bottom.Style = ExcelBorderStyle.Thin;

                    // Optionally, set the color of the borders
                    range.Style.Border.Top.Color.SetColor(System.Drawing.Color.Black);
                    range.Style.Border.Left.Color.SetColor(System.Drawing.Color.Black);
                    range.Style.Border.Right.Color.SetColor(System.Drawing.Color.Black);
                    range.Style.Border.Bottom.Color.SetColor(System.Drawing.Color.Black);

                    // Save the Excel file
                    package.Save();

                    form.Close();

                    DialogResult dialog_result = MessageBox.Show($"File stored to : {absolutePath}\n Click 'OK' to preview",
                       "Check the downloaded data",
                       MessageBoxButtons.OK,
                       MessageBoxIcon.Information);

                    if (dialog_result == DialogResult.OK)
                    {
                        try
                        {
                            Process.Start(new ProcessStartInfo
                            {
                                FileName = absolutePath,
                                UseShellExecute = true
                            });
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show($"Error opening file: \n {ex.Message}", "File Error!");
                            Console.WriteLine($"Error opening file: {ex.Message}");
                        }
                    }
                }
            }
        }

        private void inputDialogEPDM_Load(object sender, EventArgs e)
        {
            btn_verify_EPDM.Enabled = false;
        }

        private void rTBox_epdm_raw_data_TextChanged(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty(rTBox_epdm_raw_data.Text))
            {
                btn_verify_EPDM.Enabled = false;
            }
            else
            {
                btn_verify_EPDM.Enabled = true;
            }
        }
    }
}
