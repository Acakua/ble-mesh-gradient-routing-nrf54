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

# [NÂNG CẤP] Dictionary lưu trữ SET các Sequence Number để đếm Unique PDR
# Cấu trúc: { "0x00f9": {1, 2, 3...}, "0x00f8": {5, 6...} }
rx_stats = {}

# [NÂNG CẤP] Set để lưu các Node đã báo cáo xong trong phiên test này
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

def main():
    print("--- WSN LOGGER (FINAL VERSION: UNIQUE PDR & FILTER) ---")
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
                "Latency_ms",
                "PathMinRSSI", # [NEW] Thay thế RSSI chặng cuối bằng RSSI thấp nhất đường truyền
                "BeaconTx",
                "HeartbeatTx",
                "RouteChanges",
                "FwdCount",     # [NEW] Số bản tin node này đã chuyển tiếp giúp node khác
                "Rx_Unique_Count", 
                "Remote_Rx_Count", # [NEW] Số gói tin node đích nhận được (cho Downlink)
                "PDR_Percent"
            ]
            writer.writerow(headers)
            file.flush()
            
            print(f"Log file created: {filename}")
            print(f"Listening on {SERIAL_PORT}... Press Button 3 on Sink to START.")

            # Kết nối Serial
            ser = None
            while ser is None:
                try:
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                    ser.reset_input_buffer()
                    print(f"Serial Connected on {SERIAL_PORT}!")
                except serial.SerialException:
                    print(f"Waiting for {SERIAL_PORT} (Is it busy?)...")
                    time.sleep(2)

            # Vòng lặp chính
            while True:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        # Chỉ xử lý dòng chứa CSV_LOG
                        if "CSV_LOG" in line and "CSV_LOG," in line:
                            # Tách dữ liệu
                            raw_data = line.split("CSV_LOG,")[1] 
                            parts = raw_data.split(',')
                            
                            now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            log_type = parts[0]
                            
                            row = [""] * len(headers)
                            row[0] = now
                            row[1] = log_type

                            # --- XỬ LÝ GÓI TIN DATA ---
                            # Format: CSV_LOG,DATA,Src,Sender,Seq,Hops,DelayMs,PathMinRSSI
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
                                    row[6] = parts[5] # Latency
                                    row[7] = parts[6] if len(parts) > 6 else "" # PathMinRSSI

                                    # [LOGIC UNIQUE]
                                    if src_hex not in rx_stats:
                                        rx_stats[src_hex] = set()
                                    
                                    is_dup = seq_val in rx_stats[src_hex]
                                    rx_stats[src_hex].add(seq_val)
                                    
                                    dup_msg = " [DUP]" if is_dup else ""
                                    min_rssi = parts[6] if len(parts) > 6 else "N/A"
                                    print(f"[{now}] DATA | {src_hex} -> {sender_hex} | Seq:{seq_val} | MinRSSI:{min_rssi}{dup_msg}")

                            # --- XỬ LÝ GÓI TIN HEARTBEAT ---
                            # Format: CSV_LOG,HEARTBEAT,Src,Sender,Hops,DelayMs
                            elif log_type == "HEARTBEAT":
                                if len(parts) >= 5:
                                    src_val = safe_int_convert(parts[1])
                                    sender_val = safe_int_convert(parts[2])
                                    src_hex = f"0x{src_val:04x}"
                                    sender_hex = f"0x{sender_val:04x}"
                                    
                                    row[2] = src_hex
                                    row[3] = sender_hex
                                    row[5] = parts[3] # Hops
                                    row[6] = parts[4] # Latency
                                    print(f"[{now}] HEARTBEAT | Src:{src_hex} | Hops:{parts[3]}")

                            # --- XỬ LÝ GÓI TIN REPORT ---
                            # Format: CSV_LOG,REPORT,Src,Tx,Beacon,HB,RouteChanges,FwdCount
                            elif log_type == "REPORT":
                                if len(parts) >= 7:
                                    src_val = safe_int_convert(parts[1])
                                    src_hex = f"0x{src_val:04x}"

                                    if src_hex in reported_nodes:
                                        continue
                                    
                                    reported_nodes.add(src_hex)
                                    data_tx = safe_int_convert(parts[2])
                                    
                                    if src_hex in rx_stats:
                                        measured_rx = len(rx_stats[src_hex])
                                    else:
                                        measured_rx = 0

                                    pdr = (measured_rx / data_tx * 100.0) if data_tx > 0 else 0.0
                                    
                                    # [NEW] Parse Remote Rx (from parts[7])
                                    remote_rx = parts[7] if len(parts) > 7 else "0"

                                    row[2] = src_hex
                                    row[4] = str(data_tx)     # TxCount (Node)
                                    row[8] = parts[3]         # Beacon
                                    row[9] = parts[4]         # HB
                                    row[10] = parts[5]        # RouteChanges
                                    row[11] = parts[6]        # FwdCount
                                    row[12] = str(measured_rx) # Unique Count (Gateway)
                                    row[13] = str(remote_rx)   # Remote Rx (Node)
                                    row[14] = f"{pdr:.2f}"

                                    print(f"\n[{now}] >>> FINAL REPORT {src_hex} <<<")
                                    print(f"    Tx (Node):   {data_tx}")
                                    print(f"    Rx (Unique): {measured_rx}")
                                    print(f"    Rx (Remote): {remote_rx} (For Downlink)")
                                    print(f"    PDR (Up):    {pdr:.2f}%")
                                    print(f"    Fwd Count:   {parts[6]} packets")
                                    print("---------------------------------------------")

                            # --- XỬ LÝ SỰ KIỆN ---
                            elif "EVENT" in log_type:
                                event_name = parts[1]
                                msg = parts[2] if len(parts) >= 3 else ""
                                row[2] = event_name
                                row[3] = msg
                                
                                # Reset bộ đếm khi bắt đầu test mới
                                if "TEST_START" in event_name:
                                    rx_stats.clear()
                                    reported_nodes.clear()
                                    print(f"\n[{now}] --- NEW TEST SESSION STARTED (Counters Reset) ---")

                            writer.writerow(row)
                            file.flush()

                    except ValueError as e:
                        print(f"Warning: Parse error on line: {line} -> {e}")
                        continue
                    except Exception as e:
                        print(f"Error: {e}")

                else:
                    time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nLogger Stopped by User.")
    finally:
        if 'ser' in locals() and ser and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()