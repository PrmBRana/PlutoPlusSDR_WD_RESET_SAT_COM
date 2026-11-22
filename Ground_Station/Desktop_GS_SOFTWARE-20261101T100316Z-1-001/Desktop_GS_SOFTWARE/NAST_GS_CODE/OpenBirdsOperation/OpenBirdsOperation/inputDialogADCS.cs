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
    public partial class inputDialogADCS : Form
    {
        public string savingADCSRootPath;
        public string ADCSRootPath;

        public DateTime dtAppStart = DateTime.UtcNow;

        private readonly System.Text.StringBuilder _liveHexStream = new System.Text.StringBuilder();
        private bool _liveHeaderFound = false;
        private bool _liveFooterFound = false;
        private int _liveSampleCount = 0;
        private int _lastPkt = 0;
        private int _totalPkts = 0;
        private readonly System.Collections.Generic.HashSet<int> _rxPkts = new System.Collections.Generic.HashSet<int>();
        private RichTextBox rTBox_adcs_live_decoded;
        private Label lbl_adcs_live_status;

        public inputDialogADCS()
        {
            InitializeComponent();
            InitLiveDecodeUi();
        }

        private void InitLiveDecodeUi()
        {
            this.ClientSize = new System.Drawing.Size(Math.Max(this.ClientSize.Width, 680), 460);
            rTBox_adsc_raw_data.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            rTBox_adsc_raw_data.Width = this.ClientSize.Width - 18;
            rTBox_adsc_raw_data.Height = 110;

            lbl_adcs_live_status = new Label
            {
                Text = "Live Decoded ADCS Gyro (SPI4 PE4) & Mag (SPI1 PA4) — waiting for 'ADCS' (41 44 43 53)...",
                Location = new System.Drawing.Point(9, rTBox_adsc_raw_data.Bottom + 8),
                Size = new System.Drawing.Size(this.ClientSize.Width - 18, 20),
                Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
                Font = new System.Drawing.Font("Segoe UI", 9F, System.Drawing.FontStyle.Bold)
            };

            rTBox_adcs_live_decoded = new RichTextBox
            {
                Location = new System.Drawing.Point(9, lbl_adcs_live_status.Bottom + 4),
                Size = new System.Drawing.Size(this.ClientSize.Width - 18, this.ClientSize.Height - (lbl_adcs_live_status.Bottom + 56)),
                Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right,
                ReadOnly = true,
                Font = new System.Drawing.Font("Consolas", 9.5F, System.Drawing.FontStyle.Regular),
                BackColor = System.Drawing.Color.FromArgb(18, 24, 34),
                ForeColor = System.Drawing.Color.FromArgb(147, 197, 253)
            };

            btn_verify_adcs.Location = new System.Drawing.Point(this.ClientSize.Width - 210, this.ClientSize.Height - 40);
            btn_verify_adcs.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            button_cancel_adcs.Location = new System.Drawing.Point(this.ClientSize.Width - 105, this.ClientSize.Height - 40);
            button_cancel_adcs.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;

            this.Controls.Add(lbl_adcs_live_status);
            this.Controls.Add(rTBox_adcs_live_decoded);
        }

        private static short ReadInt16BE(string hex, int charOffset)
        {
            ushort val = Convert.ToUInt16(hex.Substring(charOffset, 4), 16);
            return unchecked((short)val);
        }

        private static string ExtractCleanHex(string line)
        {
            if (string.IsNullOrWhiteSpace(line)) return string.Empty;
            string s = line;
            int idx = s.IndexOf("[FLASH-HEX]", StringComparison.OrdinalIgnoreCase);
            if (idx >= 0) s = s.Substring(idx + "[FLASH-HEX]".Length);
            return Regex.Replace(s, @"[^0-9a-fA-F]", "").ToUpperInvariant();
        }

        private void UpdateLiveStatusLabel()
        {
            string pktInfo = (_totalPkts > 0)
                ? $"Pkt #{_lastPkt}/{_totalPkts} (Rx: {_rxPkts.Count}/{_totalPkts}) | "
                : "";
            if (_liveFooterFound)
                lbl_adcs_live_status.Text = $"{pktInfo}ADCS Complete ({_liveSampleCount} samples, Footer FF D9)";
            else if (_liveHeaderFound)
                lbl_adcs_live_status.Text = $"{pktInfo}Live Decoded ADCS Samples: {_liveSampleCount} (waiting for FF D9...)";
            else
                lbl_adcs_live_status.Text = $"{pktInfo}Waiting for ADCS header (41 44 43 53)...";
        }

        public void AppendLiveAdcsLine(string line)
        {
            if (this.IsDisposed) return;
            if (this.InvokeRequired)
            {
                try { this.BeginInvoke(new Action<string>(AppendLiveAdcsLine), line); } catch { }
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

            if (rTBox_adsc_raw_data.TextLength > 0) rTBox_adsc_raw_data.AppendText(Environment.NewLine);
            rTBox_adsc_raw_data.AppendText(InsertSpacesAfterEveryTwoBytes(hex));
            rTBox_adsc_raw_data.SelectionStart = rTBox_adsc_raw_data.TextLength;
            rTBox_adsc_raw_data.ScrollToCaret();

            _liveHexStream.Append(hex);
            DecodeBufferedAdcsSamples();
        }

        private void DecodeBufferedAdcsSamples()
        {
            // Stream: "ADCS" (41444353) + N x 16-byte samples (32 hex chars each) + "FFD9"
            while (!_liveFooterFound)
            {
                string buf = _liveHexStream.ToString();
                if (!_liveHeaderFound)
                {
                    int hdrIdx = buf.IndexOf("41444353", StringComparison.Ordinal);
                    if (hdrIdx < 0)
                    {
                        if (_liveHexStream.Length > 6) _liveHexStream.Remove(0, _liveHexStream.Length - 6);
                        break;
                    }
                    _liveHexStream.Remove(0, hdrIdx + 8);
                    _liveHeaderFound = true;
                    string hdrMsg = "=== ADCS HEADER RECEIVED: 41 44 43 53 ('ADCS') — Live Decoding 1 Hz Samples ===\r\n";
                    rTBox_adcs_live_decoded.AppendText(hdrMsg);
                    UpdateLiveStatusLabel();
                    Console.Write(hdrMsg);
                    continue;
                }

                if (buf.Length >= 4 && buf.StartsWith("FFD9", StringComparison.Ordinal))
                {
                    _liveHexStream.Remove(0, 4);
                    _liveFooterFound = true;
                    string ftrMsg = $"=== ADCS FOOTER RECEIVED: FF D9 | Complete! Total: {_liveSampleCount} sample(s) ===\r\n";
                    rTBox_adcs_live_decoded.AppendText(ftrMsg);
                    rTBox_adcs_live_decoded.SelectionStart = rTBox_adcs_live_decoded.TextLength;
                    rTBox_adcs_live_decoded.ScrollToCaret();
                    UpdateLiveStatusLabel();
                    Console.Write(ftrMsg);
                    break;
                }

                if (buf.Length < 32) break;

                string sHex = buf.Substring(0, 32);
                _liveHexStream.Remove(0, 32);
                _liveSampleCount++;

                ushort seq = Convert.ToUInt16(sHex.Substring(0, 4), 16);
                double gx = ReadInt16BE(sHex, 4) / 100.0;
                double gy = ReadInt16BE(sHex, 8) / 100.0;
                double gz = ReadInt16BE(sHex, 12) / 100.0;
                double gt = ReadInt16BE(sHex, 16) / 100.0;
                double mx = ReadInt16BE(sHex, 20) / 100.0;
                double my = ReadInt16BE(sHex, 24) / 100.0;
                double mz = ReadInt16BE(sHex, 28) / 100.0;
                double mb = Math.Sqrt(mx * mx + my * my + mz * mz);

                string lineOut = $"#{seq,2} | Gyro: X={gx,7:F2} Y={gy,7:F2} Z={gz,7:F2} dps (T={gt,5:F2} C) | Mag: X={mx,7:F2} Y={my,7:F2} Z={mz,7:F2} uT (|B|={mb,6:F2})\r\n";
                rTBox_adcs_live_decoded.AppendText(lineOut);
                rTBox_adcs_live_decoded.SelectionStart = rTBox_adcs_live_decoded.TextLength;
                rTBox_adcs_live_decoded.ScrollToCaret();
                UpdateLiveStatusLabel();
                Console.Write("[ADCS LIVE] " + lineOut);
            }
        }

        private void button_cancel_adcs_Click(object sender, EventArgs e)
        {
            rTBox_adsc_raw_data.Text = "";
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

        private void btn_verify_adcs_Click(object sender, EventArgs e)
        {
            try
            {
                string rawADCSRx = string.Join("", rTBox_adsc_raw_data.Lines.Select(l => ExtractCleanHex(l)));

                // Check for binary ADCS stream: 41444353 + N*32 hex chars + FFD9
                int adcsStart = rawADCSRx.IndexOf("41444353", StringComparison.Ordinal);
                if (adcsStart >= 0)
                {
                    int pos = adcsStart + 8;
                    int adcsEnd = rawADCSRx.LastIndexOf("FFD9", StringComparison.Ordinal);
                    if (adcsEnd < pos) adcsEnd = rawADCSRx.Length;
                    var samples = new System.Collections.Generic.List<string>();
                    while (pos + 32 <= adcsEnd)
                    {
                        samples.Add(rawADCSRx.Substring(pos, 32));
                        pos += 32;
                    }
                    if (samples.Count > 0)
                    {
                        savingADCSRootPath = "../Storage/ADCS_DETAILS";
                        if (!Directory.Exists(savingADCSRootPath)) Directory.CreateDirectory(savingADCSRootPath);
                        string pathXlsx = Path.Combine(savingADCSRootPath, DateTime.UtcNow.ToString("yyyyMMdd_HHmmss") + "_ADCS_Data_Verification.xlsx");
                        ExcelPackage.LicenseContext = OfficeOpenXml.LicenseContext.NonCommercial;
                        using (var pkg = new ExcelPackage(new FileInfo(pathXlsx)))
                        {
                            var ws = pkg.Workbook.Worksheets.Add("ADCS_Gyro_Mag");
                            string[] hdrs = { "Seq", "Gyro_X (dps)", "Gyro_Y (dps)", "Gyro_Z (dps)", "Gyro_Temp (C)", "Mag_X (uT)", "Mag_Y (uT)", "Mag_Z (uT)", "Mag_|B| (uT)", "Raw Hex" };
                            for (int c = 0; c < hdrs.Length; c++) ws.Cells[1, c + 1].Value = hdrs[c];
                            ws.Row(1).Style.Font.Bold = true;
                            int row = 2;
                            foreach (var sh in samples)
                            {
                                ushort sq = Convert.ToUInt16(sh.Substring(0, 4), 16);
                                double gx = ReadInt16BE(sh, 4) / 100.0;
                                double gy = ReadInt16BE(sh, 8) / 100.0;
                                double gz = ReadInt16BE(sh, 12) / 100.0;
                                double gt = ReadInt16BE(sh, 16) / 100.0;
                                double mx = ReadInt16BE(sh, 20) / 100.0;
                                double my = ReadInt16BE(sh, 24) / 100.0;
                                double mz = ReadInt16BE(sh, 28) / 100.0;
                                ws.Cells[row, 1].Value = sq;
                                ws.Cells[row, 2].Value = gx;
                                ws.Cells[row, 3].Value = gy;
                                ws.Cells[row, 4].Value = gz;
                                ws.Cells[row, 5].Value = gt;
                                ws.Cells[row, 6].Value = mx;
                                ws.Cells[row, 7].Value = my;
                                ws.Cells[row, 8].Value = mz;
                                ws.Cells[row, 9].Value = Math.Sqrt(mx * mx + my * my + mz * mz);
                                ws.Cells[row, 10].Value = InsertSpacesAfterEveryTwoBytes(sh);
                                row++;
                            }
                            ws.Cells[1, 1, row - 1, hdrs.Length].AutoFitColumns();
                            pkg.Save();
                        }
                        MessageBox.Show($"Decoded {samples.Count} ADCS sample(s).\nSaved to: {pathXlsx}", "ADCS Decoded Data", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        this.Close();
                        return;
                    }
                }

                string sequenceToExclude = "404040404040E0729C64A692406103F0";

                string result;

                // Loop to handle all occurrences of sequenceToExclude
                result = RemoveAndSeparateSequences(rawADCSRx, sequenceToExclude);

                Console.WriteLine(result);

                savingADCSRootPath = "../Storage/ADCS_DETAILS";
                Properties.Settings.Default.savePathADCS = savingADCSRootPath;
                Properties.Settings.Default.Save();

                ADCSRootPath = Properties.Settings.Default.savePathADCS;

                savingADCSRootPath = ADCSRootPath;

                if (!Directory.Exists(savingADCSRootPath))
                {
                    Directory.CreateDirectory(savingADCSRootPath);
                }

                string rawRxData = result;

                DateTime dt = DateTime.UtcNow;
                string time = dt.ToString("yyyyMMdd_HHmmss");
                string folderPath = savingADCSRootPath;
                string fileName;
                string absolutePath;

                fileName = time + "_ADCS_Data_Verification.xlsx";
                absolutePath = System.IO.Path.Combine(folderPath, fileName);
                saveToExcel(result, absolutePath, this);
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                       $"ADCS Operation Invalid \n {ex.Message}",
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
                            MessageBox.Show($"Error opening file: \n {ex.Message}",
                                "File Error!",
                                MessageBoxButtons.OK,
                                MessageBoxIcon.Warning
                            );
                            Console.WriteLine($"Error opening file: {ex.Message}");
                        }
                    }
                }
            }
        }

        private void inputDialogADCS_Load(object sender, EventArgs e)
        {
            btn_verify_adcs.Enabled = false;
        }

        private void rTBox_adsc_raw_data_TextChanged(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty(rTBox_adsc_raw_data.Text))
            {
                btn_verify_adcs.Enabled = false;
            }
            else
            {
                btn_verify_adcs.Enabled = true;
            }
        }
    }
}
