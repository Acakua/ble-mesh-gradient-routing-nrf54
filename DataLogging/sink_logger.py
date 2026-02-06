import serial
import time
import csv
import datetime
import os
import sys

# --- CẤU HÌNH ---
SERIAL_PORT = 'COM36'  # Đảm bảo đúng cổng COM của bạn
BAUD_RATE = 115200
LOG_DIR = r"C:\ncs\v3.0.1\zephyr\samples\GRADIENT-SRV\DataLogging"

# [NÂNG CẤP] Cache dữ liệu để đồng bộ Latency muộn
# Cấu trúc: { (SourceAddr, Seq): row_index }
latency_sync_cache = {}
all_rows_buffer = []

# Dictionary lưu trữ SET các Sequence Number để đếm Unique PDR
rx_stats = {}

# Set để lưu các Node đã báo cáo xong trong phiên test này
reported_nodes = set()

def get_log_filename():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(LOG_DIR, f"wsn_pdr_hops_{timestamp}.csv")

def safe_int_convert(val):
    """
    Chuyển đổi chuỗi sang số nguyên an toàn (Hỗ trợ cả Hex và Decimal).
    """
    try:
        val = str(val).strip()
        if val.lower().startswith('0x'):
            return int(val, 16)
        return int(val)
    except (ValueError, TypeError):
        return 0

def export_final_csv(current_filename, headers, rows):
    """Ghi lại file CSV cuối cùng với Latency đã được điền đầy đủ."""
    final_filename = current_filename.replace("wsn_pdr_hops_", "FINAL_LATENCY_")
    try:
        with open(final_filename, mode='w', newline='', encoding='utf-8-sig') as f:
            w = csv.writer(f)
            w.writerow(headers)
            w.writerows(rows)
        print(f"\n[FINISH] Final Latency Report exported to: {final_filename}")
    except Exception as e:
        print(f"Error exporting final CSV: {e}")

def main():
    print("--- WSN LOGGER (PING-PONG RTT VERSION) ---")
    filename = get_log_filename()
    
    if not os.path.exists(LOG_DIR): 
        os.makedirs(LOG_DIR)
    
    try:
        with open(filename, mode='w', newline='', encoding='utf-8-sig') as file:
            writer = csv.writer(file)
            
            # [HEADER]
            headers = [
                "Timestamp", 
                "Type", 
                "SourceAddr", 
                "SenderAddr", 
                "Seq_or_TxCount",
                "HopCount", 
                "RTT_ms",
                "PathMinRSSI", 
                "BeaconTx",
                "HeartbeatTx",
                "RouteChanges",
                "FwdCount",
                "Rx_Unique_Count", 
                "Remote_Rx_Count",
                "PDR_Percent"
            ]
            writer.writerow(headers)
            file.flush()
            
            print(f"Log file created: {filename}")
            print(f"Listening on {SERIAL_PORT}... Press Button 3 on Sink to START.")

            ser = None
            while ser is None:
                try:
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                    ser.reset_input_buffer()
                    print(f"Serial Connected on {SERIAL_PORT}!")
                except serial.SerialException:
                    print(f"Waiting for {SERIAL_PORT} (Is it busy?)...")
                    time.sleep(2)

            while True:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        if "CSV_LOG" in line and "CSV_LOG," in line:
                            raw_data = line.split("CSV_LOG,")[1] 
                            parts = raw_data.split(',')
                            
                            now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            log_type = parts[0]
                            
                            row = [""] * len(headers)
                            row[0] = now
                            row[1] = log_type

                            # --- XỬ LÝ GÓI TIN DATA ---
                            if log_type == "DATA":
                                if len(parts) >= 7:
                                    src_val = safe_int_convert(parts[1])
                                    sender_val = safe_int_convert(parts[2])
                                    seq_val = safe_int_convert(parts[3])
                                    
                                    src_hex = f"0x{src_val:04x}"
                                    sender_hex = f"0x{sender_val:04x}"
                                    
                                    row[2] = src_hex
                                    row[3] = sender_hex
                                    row[4] = str(seq_val)
                                    row[5] = parts[4] # Hops
                                    row[6] = "0"      # Latency (Chờ PONG report)
                                    row[7] = parts[7] if len(parts) > 7 else parts[6]
                                    
                                    # Lưu vào cache để điền Latency sau
                                    latency_sync_cache[(src_hex, seq_val)] = len(all_rows_buffer)
                                    all_rows_buffer.append(row)

                                    if src_hex not in rx_stats:
                                        rx_stats[src_hex] = set()
                                    rx_stats[src_hex].add(seq_val)
                                    
                                    print(f"[{now}] DATA | {src_hex} -> {sender_hex} | Seq:{seq_val} | Latency: Pending...")

                            # --- [NEW] XỬ LÝ RTT_DATA ---
                            # Format: CSV_LOG,RTT_DATA,Src,Seq,RTT
                            elif log_type == "RTT_DATA":
                                if len(parts) >= 4:
                                    src_hex = f"0x{safe_int_convert(parts[1]):04x}"
                                    seq_val = safe_int_convert(parts[2])
                                    rtt_val = parts[3]
                                    
                                    key = (src_hex, seq_val)
                                    if key in latency_sync_cache:
                                        row_idx = latency_sync_cache[key]
                                        all_rows_buffer[row_idx][6] = rtt_val
                                        print(f"[{now}] RTT_DATA | {src_hex} Seq {seq_val} -> {rtt_val} ms")
                                    continue # Không cần ghi dòng này vào CSV chính

                            # --- XỬ LÝ GÓI TIN HEARTBEAT ---
                            elif log_type == "HEARTBEAT":
                                if len(parts) >= 5:
                                    src_hex = f"0x{safe_int_convert(parts[1]):04x}"
                                    row[2] = src_hex
                                    row[3] = f"0x{safe_int_convert(parts[2]):04x}"
                                    row[5] = parts[3] # Hops
                                    row[6] = parts[4] # Latency
                                    all_rows_buffer.append(row)
                                    print(f"[{now}] HEARTBEAT | Src:{src_hex} | Hops:{parts[3]}")

                            # --- XỬ LÝ GÓI TIN REPORT ---
                            elif log_type == "REPORT":
                                if len(parts) >= 7:
                                    src_hex = f"0x{safe_int_convert(parts[1]):04x}"

                                    if src_hex in reported_nodes:
                                        continue
                                    
                                    reported_nodes.add(src_hex)
                                    data_tx = safe_int_convert(parts[2])
                                    measured_rx = len(rx_stats.get(src_hex, set()))
                                    pdr = (measured_rx / data_tx * 100.0) if data_tx > 0 else 0.0
                                    remote_rx = parts[7] if len(parts) > 7 else "0"

                                    row[2] = src_hex
                                    row[4] = str(data_tx)
                                    row[8] = parts[3]
                                    row[9] = parts[4]
                                    row[10] = parts[5]
                                    row[11] = parts[6]
                                    row[12] = str(measured_rx)
                                    row[13] = str(remote_rx)
                                    row[14] = f"{pdr:.2f}"
                                    
                                    all_rows_buffer.append(row)
                                    print(f"\n[{now}] >>> REPORT {src_hex} | PDR: {pdr:.2f}% <<<")

                            # --- XỬ LÝ SỰ KIỆN ---
                            elif "EVENT" in log_type:
                                event_name = parts[1]
                                msg = parts[2] if len(parts) >= 3 else ""
                                row[2] = event_name
                                row[3] = msg
                                all_rows_buffer.append(row)
                                
                                if "TEST_START" in event_name:
                                    rx_stats.clear()
                                    reported_nodes.clear()
                                    latency_sync_cache.clear()
                                    all_rows_buffer.clear()
                                    all_rows_buffer.append(row) # Giữ lại dòng TEST_START
                                    print(f"\n[{now}] --- NEW TEST SESSION STARTED ---")
                                
                                elif "TEST_STOP" in event_name:
                                    print(f"\n[{now}] --- TEST STOPPED. Generating final report... ---")
                                    export_final_csv(filename, headers, all_rows_buffer)

                            # Ghi vào file live log (vẫn ghi để backup)
                            writer.writerow(row)
                            file.flush()

                    except Exception as e:
                        print(f"Error: {e}")
                        continue
                else:
                    time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nLogger Stopped by User.")
        export_final_csv(filename, headers, all_rows_buffer)
    finally:
        if 'ser' in locals() and ser and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()